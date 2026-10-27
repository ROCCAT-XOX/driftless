#!/usr/bin/env python3
"""Independent check of a Driftless point cloud export against the LAS it came from.

    check_export.py <bezug.las> <export> [<export> …]

The reference is read with laspy, each export with a reader that has nothing to do with
Driftless: laspy/lazrs (LAS, LAZ), pye57 (E57), numpy (PLY, PCD; open3d opens them as a
second reader). Checked per file:

- the same number of points;
- the same bounds within the precision of the format: LAS/LAZ 0.1 mm, E57 and double
  PLY/PCD 1 µm, float PLY/PCD the float step at that size;
- 1000 sample points (fixed seed) with the same colour, intensity and scan number, as
  Driftless converts them (8-bit formats `round(v / 257)`, intensity 0–255 in PLY/PCD);
- E57: the points in the order of the reference sorted stably by `point_source_id`, one
  scan per entry, compared point by point after the pose.

One table row per file; the exit code is not 0 if any check fails.
"""

import sys
from pathlib import Path

import numpy as np

SAMPLES = 1000
SEED = 4711


def widen(v):
    """Driftless reads 8-bit columns (maximum ≤ 255) as × 257."""
    v = np.asarray(v, dtype=np.float64)
    return v * 257 if v.size and v.max() <= 255 else v


def read_las(path):
    import laspy

    las = laspy.read(str(path))
    d = {
        "xyz": np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], axis=1),
        "header": f"LAS {las.header.version} Format {las.header.point_format.id}",
    }
    names = set(las.point_format.dimension_names)
    if {"red", "green", "blue"} <= names:
        rgb = np.stack([las.red, las.green, las.blue], axis=1)
        d["rgb16"] = widen(rgb)
    d["intensity16"] = widen(las.intensity)
    d["source"] = np.asarray(las.point_source_id, dtype=np.int64)
    if "gps_time" in names:
        d["time"] = np.asarray(las.gps_time)
    return d


def ply_header(data):
    end = data.index(b"end_header\n") + len(b"end_header\n")
    lines = data[:end].decode("ascii").splitlines()
    assert lines[0] == "ply"
    fmt = None
    elements = []
    for line in lines[1:]:
        parts = line.split()
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            elements.append((parts[1], int(parts[2]), []))
        elif parts[0] == "property":
            elements[-1][2].append((parts[-1], parts[1:-1]))
    return end, fmt, elements


PLY_TYPES = {
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
    "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1",
    "ushort": "<u2", "uint16": "<u2", "short": "<i2", "int16": "<i2",
    "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4",
}


def read_ply(path):
    data = path.read_bytes()
    end, fmt, elements = ply_header(data)
    assert fmt == "binary_little_endian", fmt
    name, count, props = elements[0]
    assert name == "vertex"
    dtype = np.dtype([(p, PLY_TYPES[t[0]]) for p, t in props])
    v = np.frombuffer(data, dtype=dtype, count=count, offset=end)
    d = {
        "xyz": np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64),
        "header": "PLY xyz " + props[0][1][0],
        "double": props[0][1][0] == "double",
    }
    if "red" in v.dtype.names:
        d["rgb8"] = np.stack([v["red"], v["green"], v["blue"]], axis=1).astype(np.float64)
    if "intensity" in v.dtype.names:
        d["intensity255"] = v["intensity"].astype(np.float64)
    if "scan" in v.dtype.names:
        d["source"] = v["scan"].astype(np.int64)
    return d


def read_pcd(path):
    data = path.read_bytes()
    header = {}
    pos = 0
    while True:
        nl = data.index(b"\n", pos)
        line = data[pos:nl].decode("ascii").strip()
        pos = nl + 1
        if not line or line.startswith("#"):
            continue
        key, *rest = line.split()
        header[key] = rest
        if key == "DATA":
            break
    assert header["DATA"] == ["binary"], header["DATA"]
    types = {("F", "4"): "<f4", ("F", "8"): "<f8", ("U", "2"): "<u2", ("U", "4"): "<u4"}
    dtype = np.dtype(
        [(f, types[(t, s)]) for f, t, s in zip(header["FIELDS"], header["TYPE"], header["SIZE"])]
    )
    n = int(header["POINTS"][0])
    v = np.frombuffer(data, dtype=dtype, count=n, offset=pos)
    d = {
        "xyz": np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64),
        "header": "PCD xyz " + header["TYPE"][0] + header["SIZE"][0],
        "double": header["SIZE"][0] == "8",
    }
    if "rgb" in v.dtype.names:
        bits = v["rgb"].view("<u4") if v["rgb"].dtype != np.dtype("<u4") else v["rgb"]
        d["rgb8"] = np.stack(
            [(bits >> 16) & 255, (bits >> 8) & 255, bits & 255], axis=1
        ).astype(np.float64)
    if "intensity" in v.dtype.names:
        d["intensity255"] = v["intensity"].astype(np.float64)
    if "scan" in v.dtype.names:
        d["source"] = v["scan"].astype(np.int64)
    return d


def open3d_count(path):
    """open3d as a second reader: the number of points it sees (or None without open3d)."""
    try:
        import open3d as o3d
    except ImportError:
        return None
    return len(o3d.io.read_point_cloud(str(path)).points)


def read_e57(path):
    import pye57

    # pye57 reads colours into 8-bit buffers; 16-bit colours (limits 0–65535, which E57
    # allows and Driftless writes for colours not of 8-bit origin) need wider ones.
    for c in ("colorRed", "colorGreen", "colorBlue"):
        pye57.e57.SUPPORTED_POINT_FIELDS[c] = "H"
    e57 = pye57.E57(str(path))
    xyz, rgb, inten, src, names = [], [], [], [], []
    for k in range(e57.scan_count):
        s = e57.read_scan(k, colors=True, intensity=True, transform=True,
                          ignore_missing_fields=True)
        n = len(s["cartesianX"])
        xyz.append(np.stack([s["cartesianX"], s["cartesianY"], s["cartesianZ"]], axis=1))
        if "colorRed" in s:
            rgb.append(np.stack([s["colorRed"], s["colorGreen"], s["colorBlue"]], axis=1))
        if "intensity" in s:
            inten.append(np.asarray(s["intensity"], dtype=np.float64))
        src.append(np.full(n, k))
        names.append(e57.get_header(k).node["name"].value())
    header = e57.get_header(0)
    d = {"xyz": np.concatenate(xyz), "source": np.concatenate(src), "names": names,
         "header": f"E57 {e57.scan_count} Scans"}
    if rgb:
        rgb = np.concatenate(rgb).astype(np.float64)
        # pye57 gives the stored integers; their limits are 0–255 or 0–65535.
        limit = header.node["colorLimits"]["colorRedMaximum"].value()
        d["rgb16"] = rgb * (65535.0 / limit)
    if inten:
        inten = np.concatenate(inten)
        limit = header.node["intensityLimits"]["intensityMaximum"].value()
        d["intensity16"] = inten * (65535.0 / limit)
    return d


def float_step(xyz):
    return np.abs(xyz).max() * 2.0**-23 + 1e-9


def check(ref, path):
    ext = path.suffix.lower()
    fails = []
    if ext in (".las", ".laz"):
        exp = read_las(path)
        tol = 1e-4 + 1e-9
    elif ext == ".e57":
        exp = read_e57(path)
        tol = 1e-6
        order = np.argsort(ref["source"], kind="stable")
        ref = {k: (v[order] if isinstance(v, np.ndarray) else v) for k, v in ref.items()}
        want = np.unique(ref["source"]).size
        if len(exp["names"]) != want:
            fails.append(f"{len(exp['names'])} Scans statt {want}")
    elif ext == ".ply":
        exp = read_ply(path)
        tol = 1e-6 if exp["double"] else float_step(ref["xyz"])
    elif ext == ".pcd":
        exp = read_pcd(path)
        tol = 1e-6 if exp["double"] else float_step(ref["xyz"])
    else:
        return [path.name, "?", "-", "-", f"unbekannte Endung {ext}"]

    n_ref, n_exp = len(ref["xyz"]), len(exp["xyz"])
    if n_ref != n_exp:
        fails.append(f"Zahl {n_exp} statt {n_ref}")
        return [path.name, exp["header"], n_exp, "-", "; ".join(fails)]
    if ext in (".ply", ".pcd"):
        o3d = open3d_count(path)
        if o3d is not None and o3d != n_ref:
            fails.append(f"open3d liest {o3d} Punkte")

    bmax = max(
        np.abs(ref["xyz"].min(axis=0) - exp["xyz"].min(axis=0)).max(),
        np.abs(ref["xyz"].max(axis=0) - exp["xyz"].max(axis=0)).max(),
    ) if n_ref else 0.0
    if bmax > tol:
        fails.append(f"Grenzen weichen {bmax:.3g} m ab (erlaubt {tol:.3g})")

    rng = np.random.default_rng(SEED)
    idx = rng.choice(n_ref, size=min(SAMPLES, n_ref), replace=False) if n_ref else []
    dmax = np.abs(ref["xyz"][idx] - exp["xyz"][idx]).max() if n_ref else 0.0
    if dmax > tol:
        fails.append(f"Stichproben weichen {dmax:.3g} m ab")
    if "rgb16" in ref:
        if "rgb16" in exp:
            d = np.abs(ref["rgb16"][idx] - exp["rgb16"][idx]).max() if n_ref else 0
            if d > 0.5:
                fails.append(f"Farbe weicht {d} ab (16 Bit)")
        elif "rgb8" in exp:
            d = np.abs(np.round(ref["rgb16"][idx] / 257) - exp["rgb8"][idx]).max() if n_ref else 0
            if d > 0:
                fails.append(f"Farbe weicht {d} ab (8 Bit)")
        else:
            fails.append("Farbe fehlt")
    if "intensity16" in ref and ref["intensity16"].any():
        if "intensity16" in exp:
            d = np.abs(ref["intensity16"][idx] - exp["intensity16"][idx]).max()
            if d > 1.0:
                fails.append(f"Intensität weicht {d} ab")
        elif "intensity255" in exp:
            d = np.abs(ref["intensity16"][idx] / 257 - exp["intensity255"][idx]).max()
            if d > 0.01:
                fails.append(f"Intensität weicht {d:.3g} ab (0–255)")
        else:
            fails.append("Intensität fehlt")
    if np.unique(ref["source"]).size > 1:
        if "source" not in exp:
            fails.append("Scan-Nummer fehlt")
        else:
            want = ref["source"][idx]
            if ext == ".e57":
                # Entries in the order of the scan numbers that occur.
                want = np.searchsorted(np.unique(ref["source"]), want)
            if not np.array_equal(want, exp["source"][idx]):
                fails.append("Scan-Nummer weicht ab")
    return [path.name, exp["header"], n_exp, f"{max(bmax, dmax):.2g}",
            "ok" if not fails else "; ".join(fails)]


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    ref = read_las(Path(sys.argv[1]))
    rows = [["Datei", "Kopf", "Punkte", "max. Abw. m", "Ergebnis"]]
    bad = 0
    for p in sys.argv[2:]:
        try:
            row = check(ref, Path(p))
        except Exception as e:  # noqa: BLE001 – a reader failing is a failed check
            row = [Path(p).name, "?", "-", "-", f"nicht lesbar: {e}"]
        bad += row[-1] != "ok"
        rows.append([str(c) for c in row])
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    for r in rows:
        print(" | ".join(c.ljust(w) for c, w in zip(r, widths)))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
