// glim_driftless: feeds the streams written by `driftless export-glim` into
// GLIM (odometry -> sub mapping -> global mapping) without ROS and saves the
// result with GlobalMapping::save (odom_imu.txt, traj_imu.txt in TUM format).
//
// Usage: glim_driftless <config_dir> <input_dir> <dump_dir> [module_dir]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <glim/util/config.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace fs = std::filesystem;

#ifndef GLIM_MODULE_DIR
#define GLIM_MODULE_DIR ""
#endif

struct Imu {
  double stamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
};

static bool check_magic(std::ifstream& in, const char* magic) {
  char buf[8];
  in.read(buf, 8);
  return in && std::string(buf, 8) == std::string(magic, 8);
}

static std::vector<Imu> read_imu(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!check_magic(in, "DLIMU01\0")) {
    throw std::runtime_error("bad imu.bin: " + path.string());
  }
  std::vector<Imu> out;
  double v[7];
  while (in.read(reinterpret_cast<char*>(v), sizeof(v))) {
    out.push_back({v[0], Eigen::Vector3d(v[1], v[2], v[3]), Eigen::Vector3d(v[4], v[5], v[6])});
  }
  std::stable_sort(out.begin(), out.end(), [](const Imu& a, const Imu& b) { return a.stamp < b.stamp; });
  return out;
}

#pragma pack(push, 1)
struct PointRec {
  float x, y, z, intensity, t_rel;
  uint32_t ring;
};
#pragma pack(pop)
static_assert(sizeof(PointRec) == 24, "point record layout");

/// Reads the next frame; returns nullptr at end of file.
static glim::RawPoints::Ptr read_frame(std::ifstream& in) {
  double stamp;
  uint32_t n;
  if (!in.read(reinterpret_cast<char*>(&stamp), 8) || !in.read(reinterpret_cast<char*>(&n), 4)) {
    return nullptr;
  }
  std::vector<PointRec> recs(n);
  if (n && !in.read(reinterpret_cast<char*>(recs.data()), n * sizeof(PointRec))) {
    throw std::runtime_error("lidar.bin truncated");
  }
  auto raw = std::make_shared<glim::RawPoints>();
  raw->stamp = stamp;
  raw->points.reserve(n);
  raw->times.reserve(n);
  raw->intensities.reserve(n);
  raw->rings.reserve(n);
  for (const auto& r : recs) {
    raw->points.emplace_back(r.x, r.y, r.z, 1.0);
    raw->times.push_back(r.t_rel);
    raw->intensities.push_back(r.intensity);
    raw->rings.push_back(r.ring);
  }
  return raw;
}

/// GLIM configs name modules "lib<name>.so"; map to the platform's file.
static std::string module_path(const std::string& module_dir, std::string so_name) {
  const std::string so = ".so";
  if (so_name.size() > so.size() && so_name.compare(so_name.size() - so.size(), so.size(), so) == 0) {
    so_name.resize(so_name.size() - so.size());
#if defined(_WIN32)
    if (so_name.rfind("lib", 0) == 0) so_name = so_name.substr(3);
    so_name += ".dll";
#elif defined(__APPLE__)
    so_name += ".dylib";
#else
    so_name += ".so";
#endif
  }
  return module_dir.empty() ? so_name : (fs::path(module_dir) / so_name).string();
}

static double peak_rss_mb() {
#ifdef _WIN32
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
  return pmc.PeakWorkingSetSize / 1048576.0;
#else
  rusage ru;
  getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
  return ru.ru_maxrss / 1048576.0;  // bytes on macOS
#else
  return ru.ru_maxrss / 1024.0;  // kilobytes on Linux
#endif
#endif
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: glim_driftless <config_dir> <input_dir> <dump_dir> [module_dir]" << std::endl;
    return 2;
  }
  const std::string config_dir = fs::absolute(argv[1]).string();
  const fs::path input_dir = argv[2];
  const std::string dump_dir = fs::absolute(argv[3]).string();
  const std::string module_dir = argc > 4 ? argv[4] : GLIM_MODULE_DIR;
  fs::create_directories(dump_dir);

  auto logger = spdlog::stdout_color_mt("glim");
  spdlog::set_default_logger(logger);
  spdlog::set_level(spdlog::level::info);
  const auto t0 = std::chrono::steady_clock::now();

  glim::GlobalConfig::instance(config_dir);
  const auto so = [&](const char* config, const char* module, const char* fallback) {
    return module_path(module_dir, glim::Config(glim::GlobalConfig::get_config_path(config)).param<std::string>(module, "so_name", fallback));
  };

  auto odom = glim::OdometryEstimationBase::load_module(so("config_odometry", "odometry_estimation", "libodometry_estimation_cpu.so"));
  if (!odom) {
    spdlog::critical("failed to load odometry module");
    return 1;
  }
  glim::AsyncOdometryEstimation odometry(odom, odom->requires_imu());
  auto sub = glim::SubMappingBase::load_module(so("config_sub_mapping", "sub_mapping", "libsub_mapping.so"));
  auto global = glim::GlobalMappingBase::load_module(so("config_global_mapping", "global_mapping", "libglobal_mapping.so"));
  if (!sub || !global) {
    spdlog::critical("failed to load mapping modules");
    return 1;
  }
  glim::AsyncSubMapping sub_mapping(sub);
  glim::AsyncGlobalMapping global_mapping(global);

  glim::TimeKeeper time_keeper;
  glim::CloudPreprocessor preprocessor;

  const auto imu = read_imu(input_dir / "imu.bin");
  spdlog::info("{} IMU samples", imu.size());
  std::ifstream lidar(input_dir / "lidar.bin", std::ios::binary);
  if (!check_magic(lidar, "DLPTS01\0")) {
    spdlog::critical("bad lidar.bin");
    return 1;
  }

  const auto forward_results = [&]() {
    std::vector<glim::EstimationFrame::ConstPtr> estimated, marginalized;
    odometry.get_results(estimated, marginalized);
    for (const auto& f : marginalized) sub_mapping.insert_frame(f);
    for (const auto& s : sub_mapping.get_results()) global_mapping.insert_submap(s);
  };

  size_t next_imu = 0, frames = 0, skipped = 0;
  while (auto raw = read_frame(lidar)) {
    // IMU must cover the whole scan (relative point times up to ~0.1 s).
    for (; next_imu < imu.size() && imu[next_imu].stamp <= raw->stamp + 0.2; next_imu++) {
      const auto& s = imu[next_imu];
      if (!time_keeper.validate_imu_stamp(s.stamp)) continue;
      odometry.insert_imu(s.stamp, s.acc, s.gyro);
      sub_mapping.insert_imu(s.stamp, s.acc, s.gyro);
      global_mapping.insert_imu(s.stamp, s.acc, s.gyro);
    }
    if (raw->points.empty() || !time_keeper.process(raw)) {
      skipped++;
      continue;
    }
    odometry.insert_frame(preprocessor.preprocess(raw));
    frames++;
    // Offline playback: do not run ahead of the estimator.
    while (odometry.workload() > 10) {
      forward_results();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    forward_results();
    if (frames % 200 == 0) spdlog::info("frames={}", frames);
  }
  for (; next_imu < imu.size(); next_imu++) {
    const auto& s = imu[next_imu];
    odometry.insert_imu(s.stamp, s.acc, s.gyro);
  }

  spdlog::info("waiting for odometry ({} frames, {} skipped)", frames, skipped);
  odometry.join();
  forward_results();
  spdlog::info("waiting for sub mapping");
  sub_mapping.join();
  for (const auto& s : sub_mapping.get_results()) global_mapping.insert_submap(s);
  spdlog::info("waiting for global mapping");
  global_mapping.join();
  global_mapping.save(dump_dir);

  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("frames=%zu skipped=%zu runtime_s=%.1f peak_rss_mb=%.0f\n", frames, skipped, secs, peak_rss_mb());
  return 0;
}
