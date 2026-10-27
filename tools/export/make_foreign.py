#!/usr/bin/env python3
"""Small foreign sample files for `crates/driftless-export/tests/foreign.rs`.

Writes a synthetic scene (fixed seed: 600 points on three planes, one colour per plane)
with the libraries other programs use, and `expected.json` with what Driftless must read
from each file: the number of points (or triangles), the bounds (Z up, metres) and the
colour of the first five points (16 bit, as Driftless keeps it: 8-bit values × 257).

    make_foreign.py <ordner>

Libraries: laspy (+ lazrs), pye57, open3d, trimesh, numpy, Pillow. No scan data.
"""

import json
import sys
from pathlib import Path

import numpy as np

SEED = 57
PER_PLANE = 200
# One colour per plane (8 bit).
COLOURS = np.array([[200, 60, 40], [40, 180, 70], [50, 80, 210]], dtype=np.uint16)


def scene():
    """xyz (n × 3, f64), rgb8 (n × 3), intensity8 (n), plane (n)."""
    rng = np.random.default_rng(SEED)
    parts = []
    for plane in range(3):
        a = rng.uniform(0.0, 4.0, PER_PLANE)
        b = rng.uniform(0.0, 2.5, PER_PLANE)
        zero = np.zeros(PER_PLANE)
        # Floor z = 0, wall x = 0, wall y = 3; a little noise across each plane.
        noise = rng.normal(0.0, 0.002, PER_PLANE)
        if plane == 0:
            xyz = np.stack([a, b, zero + noise], axis=1)
        elif plane == 1:
            xyz = np.stack([zero + noise, a * 0.75, b], axis=1)
        else:
            xyz = np.stack([a, zero + 3.0 + noise, b], axis=1)
        parts.append(xyz)
    xyz = np.round(np.concatenate(parts), 4)
    plane = np.repeat(np.arange(3), PER_PLANE)
    rgb8 = COLOURS[plane]
    intensity8 = rng.integers(10, 250, len(xyz)).astype(np.uint16)
    return xyz, rgb8, intensity8, plane


def bounds(xyz):
    return [xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()]


def entry(xyz, rgb16, **extra):
    out = {"count": int(len(xyz)), "bounds": bounds(xyz), "rgb": rgb16[:5].tolist()}
    out.update(extra)
    return out


def write_laspy(out, xyz, rgb8, intensity8, plane, expected):
    import laspy

    # LAS 1.2, format 2, colours as 8-bit values (as some programs write them).
    h = laspy.LasHeader(point_format=2, version="1.2")
    h.scales = [0.0001] * 3
    h.offsets = [0.0, 0.0, 0.0]
    las = laspy.LasData(h)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    las.red, las.green, las.blue = rgb8[:, 0], rgb8[:, 1], rgb8[:, 2]
    las.intensity = intensity8
    las.write(out / "laspy_12_8bit.las")
    expected["laspy_12_8bit.las"] = entry(xyz, rgb8 * 257, tolerance=1e-4)

    # LAS 1.4, format 7, compressed: 16-bit colours, times, the plane as point_source_id.
    h = laspy.LasHeader(point_format=7, version="1.4")
    h.scales = [0.0001] * 3
    h.offsets = [100.0, 200.0, 0.0]
    las = laspy.LasData(h)
    moved = xyz + np.array([100.0, 200.0, 0.0])
    las.x, las.y, las.z = moved[:, 0], moved[:, 1], moved[:, 2]
    rgb16 = rgb8 * 257
    las.red, las.green, las.blue = rgb16[:, 0], rgb16[:, 1], rgb16[:, 2]
    las.intensity = intensity8 * 257
    las.gps_time = 1.7e9 + np.arange(len(xyz)) * 1e-3
    las.point_source_id = plane
    las.write(out / "laspy_14_fmt7.laz", laz_backend=laspy.LazBackend.Lazrs)
    expected["laspy_14_fmt7.laz"] = entry(moved, rgb16, tolerance=1e-4, sources=3)


def quat_from_yaw(yaw):
    return [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]  # w x y z


def rotate(q, p):
    w, x, y, z = q
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    return p @ r.T


def write_pye57(out, xyz, rgb8, intensity8, plane, expected, notes):
    import pye57

    # Two scans with a pose each: the points in the scan frame, the pose to the file frame.
    path = out / "pye57_two_scans.e57"
    if path.exists():
        path.unlink()
    e57 = pye57.E57(str(path), mode="w")
    file_frame = []
    rgb_order = []
    poses = [([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]), (quat_from_yaw(0.5), [2.0, -1.0, 0.5])]
    for k, (q, t) in enumerate(poses):
        sel = plane == k  # planes 0 and 1
        local = xyz[sel]
        data = {
            "cartesianX": local[:, 0],
            "cartesianY": local[:, 1],
            "cartesianZ": local[:, 2],
            "colorRed": rgb8[sel, 0].astype(np.uint8),
            "colorGreen": rgb8[sel, 1].astype(np.uint8),
            "colorBlue": rgb8[sel, 2].astype(np.uint8),
            "intensity": intensity8[sel].astype(np.float32) / 255.0,
        }
        e57.write_scan_raw(
            data, name=f"Ebene {k + 1}", rotation=np.array(q), translation=np.array(t)
        )
        file_frame.append(rotate(q, local) + np.array(t))
        rgb_order.append(rgb8[sel] * 257)
    e57.close()
    expected["pye57_two_scans.e57"] = entry(
        np.concatenate(file_frame),
        np.concatenate(rgb_order),
        tolerance=1e-6,
        sources=2,
        scans=["Ebene 1", "Ebene 2"],
    )

    # One scan with spherical coordinates only.
    path = out / "pye57_spherical.e57"
    if path.exists():
        path.unlink()
    sel = plane == 2
    p = xyz[sel]
    rng_ = np.linalg.norm(p, axis=1)
    az = np.arctan2(p[:, 1], p[:, 0])
    el = np.arcsin(p[:, 2] / rng_)
    try:
        e57 = pye57.E57(str(path), mode="w")
        e57.write_scan_raw(
            {
                "sphericalRange": rng_,
                "sphericalAzimuth": az,
                "sphericalElevation": el,
                "colorRed": rgb8[sel, 0].astype(np.uint8),
                "colorGreen": rgb8[sel, 1].astype(np.uint8),
                "colorBlue": rgb8[sel, 2].astype(np.uint8),
            },
            name="Kugel",
        )
        e57.close()
        expected["pye57_spherical.e57"] = entry(p, rgb8[sel] * 257, tolerance=1e-6)
    except Exception as e:  # noqa: BLE001 – report, not fail: pye57 may refuse
        notes.append(f"pye57 schreibt keine nur sphärische Datei: {e}")
        if path.exists():
            path.unlink()


def write_open3d(out, xyz, rgb8, expected):
    import open3d as o3d

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)
    pc.colors = o3d.utility.Vector3dVector(rgb8 / 255.0)
    for name, kw in [
        ("open3d_ascii.pcd", {"write_ascii": True}),
        ("open3d_binary.pcd", {}),
        ("open3d_binary_compressed.pcd", {"compressed": True}),
        ("open3d.ply", {}),
    ]:
        ok = o3d.io.write_point_cloud(str(out / name), pc, **kw)
        assert ok, name
        # open3d keeps xyz as given (double in PLY, float in PCD).
        tol = 1e-6 if name.endswith(".ply") else 1e-5
        expected[name] = entry(xyz, rgb8 * 257, tolerance=tol)
    expected["open3d_binary_compressed.pcd"] = {
        "error": "PCD „binary_compressed“ wird nicht unterstützt – in CloudCompare oder PCL "
        "als „binary“ speichern"
    }


def texture_png():
    from PIL import Image

    img = Image.new("RGB", (8, 8))
    for y in range(8):
        for x in range(8):
            img.putpixel((x, y), (30 * x, 30 * y, 128))
    return img


def write_trimesh(out, expected):
    import trimesh

    # A quad (two triangles) and a second quad, with uvs and a PNG texture.
    v = np.array(
        [[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0], [0, 0, 1], [2, 0, 1], [2, 1, 1]],
        dtype=np.float64,
    )
    f = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6]])
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0], [1, 0], [1, 1]], dtype=np.float64)
    material = trimesh.visual.material.SimpleMaterial(image=texture_png())
    mesh = trimesh.Trimesh(
        vertices=v,
        faces=f,
        visual=trimesh.visual.TextureVisuals(uv=uv, material=material),
        process=False,
    )

    # glb (Y up): a node that moves the mesh by (1, 2, 3) in the glTF frame.
    scene = trimesh.Scene()
    scene.add_geometry(
        mesh, node_name="verschoben", transform=trimesh.transformations.translation_matrix([1, 2, 3])
    )
    (out / "trimesh_node.glb").write_bytes(scene.export(file_type="glb"))
    # In Driftless (Z up): glTF (x, y, z) → (x, −z, y).
    gl = v + np.array([1.0, 2.0, 3.0])
    zup = np.stack([gl[:, 0], -gl[:, 2], gl[:, 1]], axis=1)
    expected["trimesh_node.glb"] = {
        "triangles": 3,
        "bounds": bounds(zup),
        "image": "png",
        "tolerance": 1e-6,
    }

    # OBJ with .mtl and PNG, written as Z up (taken as it is).
    files = trimesh.exchange.obj.export_obj(
        mesh, include_texture=True, return_texture=True, mtl_name="trimesh_obj.mtl"
    )
    text, textures = files
    (out / "trimesh_obj.obj").write_text(text)
    for name, data in textures.items():
        (out / name).write_bytes(data)
    expected["trimesh_obj.obj"] = {
        "triangles": 3,
        "bounds": bounds(v),
        "image": "png",
        "tolerance": 1e-6,
        "files": sorted(textures),
    }


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    xyz, rgb8, intensity8, plane = scene()
    expected = {}
    notes = []
    write_laspy(out, xyz, rgb8, intensity8, plane, expected)
    write_pye57(out, xyz, rgb8, intensity8, plane, expected, notes)
    write_open3d(out, xyz, rgb8, expected)
    write_trimesh(out, expected)
    expected = {k: expected[k] for k in sorted(expected)}
    (out / "expected.json").write_text(json.dumps(expected, indent=1, ensure_ascii=False) + "\n")
    total = sum(p.stat().st_size for p in out.iterdir())
    print(f"{len(expected)} Dateien, zusammen {total / 1024:.0f} KB")
    for n in notes:
        print("Hinweis:", n)
    if total > 300 * 1024:
        sys.exit("zu groß (höchstens 300 KB)")


if __name__ == "__main__":
    main()
