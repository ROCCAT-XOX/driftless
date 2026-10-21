cmake_minimum_required(VERSION 3.22)
project(glim_driftless LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(glim REQUIRED)
find_package(spdlog REQUIRED)

add_executable(glim_driftless main.cpp)
target_link_libraries(glim_driftless glim::glim spdlog::spdlog)
# Directory with the dynamically loaded estimation modules.
target_compile_definitions(glim_driftless PRIVATE GLIM_MODULE_DIR="${GLIM_MODULE_DIR}")
if(WIN32)
  target_link_libraries(glim_driftless psapi)
endif()
install(TARGETS glim_driftless RUNTIME DESTINATION bin)
