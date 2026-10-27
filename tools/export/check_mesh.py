#!/usr/bin/env python3
"""Independent check of a Driftless mesh export against the glb it came from.

    check_mesh.py <oberflaeche.glb> <export> [<export> …]

Read with trimesh (nothing of Driftless). Checked per file:

- glb and OBJ: the same number of corners and triangles as the glb (for OBJ the corners
  are its `v` lines: trimesh splits corners by texture coordinate);
- PLY mesh: the same number of triangles and as many corners as the glb has distinct
  positions (the PLY mesh welds equal corners of neighbouring pages);
- the bounds after the change of axes: glb is Y up, OBJ and PLY Z up, a Driftless point
  (x, y, z) is (x, z, −y) in glTF; within 1 mm (glb stores float);
- PLY mesh: the mean corner colour within ±3 of the mean texture colour of the glb at its
  distinct positions (sampled here with numpy, bilinear between texel centres, and
  averaged over the pages that share a position).

One table row per file; the exit code is not 0 if any check fails.
"""

import sys
from pathlib import Path

import numpy as np
import trimesh

TOL = 1e-3


def meshes(path, **kw):
    """All geometry of a file, with the node transforms applied."""
    loaded = trimesh.load(str(path), process=False, maintain_order=True, **kw)
    if isinstance(loaded, trimesh.Scene):
        return [g for g in loaded.dump(concatenate=False) if isinstance(g, trimesh.Trimesh)]
    return [loaded]


def counts(parts):
    return sum(len(m.vertices) for m in parts), sum(len(m.faces) for m in parts)


def bounds(parts):
    v = np.concatenate([m.vertices for m in parts]) if parts else np.zeros((0, 3))
    return (v.min(axis=0), v.max(axis=0)) if len(v) else (np.zeros(3), np.zeros(3))


def zup_to_gltf(lo, hi):
    """Bounds of Z-up points in the glTF frame: (x, y, z) → (x, z, −y)."""
    return (np.array([lo[0], lo[2], -hi[1]]), np.array([hi[0], hi[2], -lo[1]]))


def sample(image, uv):
    """Bilinear samples of `image` (H × W × 3) at trimesh uvs (v up), between texel centres."""
    h, w = image.shape[:2]
    x = np.clip(uv[:, 0] * w - 0.5, 0, w - 1)
    y = np.clip((1.0 - uv[:, 1]) * h - 0.5, 0, h - 1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0 + 1, w - 1), np.minimum(y0 + 1, h - 1)
    fx, fy = (x - x0)[:, None], (y - y0)[:, None]
    top = image[y0, x0] * (1 - fx) + image[y0, x1] * fx
    bottom = image[y1, x0] * (1 - fx) + image[y1, x1] * fx
    return top * (1 - fy) + bottom * fy


def mean_texture_colour(parts):
    """Mean over the distinct positions of the mean texture colour at each."""
    pos, col = [], []
    for m in parts:
        vis = m.visual
        mat = getattr(vis, "material", None)
        img = getattr(mat, "baseColorTexture", None) or getattr(mat, "image", None)
        if img is not None and getattr(vis, "uv", None) is not None:
            c = sample(np.asarray(img.convert("RGB"), dtype=np.float64), np.asarray(vis.uv))
        else:
            factor = getattr(mat, "baseColorFactor", None)
            rgb = np.asarray(factor[:3] if factor is not None else [255, 255, 255], float)
            c = np.tile(rgb, (len(m.vertices), 1))
        used = np.unique(m.faces)
        pos.append(np.asarray(m.vertices)[used])
        col.append(c[used])
    if not pos:
        return np.zeros(3)
    pos, col = np.concatenate(pos), np.concatenate(col)
    _, inverse = np.unique(pos, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    n = inverse.max() + 1
    sums = np.zeros((n, 3))
    np.add.at(sums, inverse, col)
    per = sums / np.bincount(inverse, minlength=n)[:, None]
    return per.mean(axis=0)


def check(ref_parts, path):
    ext = path.suffix.lower()
    fails = []
    ref_v, ref_f = counts(ref_parts)
    ref_lo, ref_hi = bounds(ref_parts)
    if ext == ".glb":
        parts = meshes(path)
        lo, hi = bounds(parts)
        kind = "glb"
    elif ext == ".obj":
        parts = meshes(path)
        lo, hi = zup_to_gltf(*bounds(parts))
        kind = "OBJ"
    elif ext == ".ply":
        parts = meshes(path)
        lo, hi = zup_to_gltf(*bounds(parts))
        kind = "PLY-Netz"
    else:
        return [path.name, "?", "-", "-", f"unbekannte Endung {ext}"]
    v, f = counts(parts)
    if ext == ".obj":
        with open(path, "rb") as fh:
            v = sum(1 for line in fh if line.startswith(b"v "))
    if ext == ".ply":
        distinct = len(np.unique(np.concatenate([m.vertices for m in ref_parts]), axis=0))
        if v != distinct:
            fails.append(f"{v} Ecken statt {distinct} verschiedener")
        want = mean_texture_colour(ref_parts)
        got = np.asarray(parts[0].visual.vertex_colors)[:, :3].astype(np.float64).mean(axis=0)
        if np.abs(want - got).max() > 3:
            fails.append(f"Mittelfarbe {got.round(1)} statt {want.round(1)}")
    elif v != ref_v:
        fails.append(f"{v} Ecken statt {ref_v}")
    if f != ref_f:
        fails.append(f"{f} Dreiecke statt {ref_f}")
    dev = max(np.abs(lo - ref_lo).max(), np.abs(hi - ref_hi).max())
    if dev > TOL:
        fails.append(f"Grenzen weichen {dev:.3g} m ab")
    return [path.name, kind, f"{v}/{f}", f"{dev:.2g}", "ok" if not fails else "; ".join(fails)]


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    ref = meshes(Path(sys.argv[1]))
    rows = [["Datei", "Art", "Ecken/Dreiecke", "max. Abw. m", "Ergebnis"]]
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
