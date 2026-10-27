# Independent check of the exports

These scripts check what `driftless export` writes with libraries that share no code with
Driftless. They run on VM 101 (Python in the user account, no sudo), not in the CI.

## Setup

```sh
python3 -m venv ~/venv-export            # Ubuntu 24.04 refuses `pip install --user` (PEP 668)
~/venv-export/bin/pip install laspy lazrs pye57 open3d trimesh scipy numpy pillow
```

Tested with laspy 2.7.0, lazrs 0.8.2, pye57 0.4.19, open3d 0.20.0, trimesh 5.2.0,
numpy 2.5.4 (Python 3.12).

## Scripts

| Script | Call | What it checks |
|---|---|---|
| `check_export.py` | `check_export.py <bezug.las> <export>…` | Point clouds (LAS, LAZ, E57, PLY, PCD) against the LAS they came from |
| `check_mesh.py` | `check_mesh.py <oberflaeche.glb> <export>…` | Meshes (glb, OBJ, PLY mesh) against the glb they came from |
| `make_foreign.py` | `make_foreign.py <ordner>` | Writes the foreign sample files and `expected.json` |

Each check prints one table row per file and exits with a code other than 0 if any file
deviates.

### `check_export.py`

- Reference: laspy. Exports: laspy/lazrs (LAS, LAZ), pye57 (E57), own numpy readers for
  binary PLY and PCD, with open3d as a second reader for the number of points.
- Same number of points.
- Bounds within the precision of the format: LAS/LAZ 0.1 mm, E57 and PLY/PCD with
  `double` 1 µm, PLY/PCD with `float` the float step at that size.
- 1000 sample points (fixed seed): position, colour, intensity and scan number, converted
  as Driftless does (8-bit formats `round(v / 257)`, intensity 0–255 in PLY/PCD, 8-bit
  LAS columns read × 257).
- E57: the points in the order of the reference sorted stably by `point_source_id`, one
  entry per scan, compared after the pose of each entry.
- E57 colours are written as 8 bit (limits 0–255), as pye57 and other libE57-based tools
  expect. They are compared as `round(v / 257)`. The script still widens pye57's colour
  buffers, so that foreign E57 files with 16-bit colours can be read too.

### `check_mesh.py`

- Read with trimesh.
- glb and OBJ have the same number of corners and triangles as the glb. For OBJ the
  corners are its `v` lines, because trimesh splits corners by texture coordinate.
- The PLY mesh has the same number of triangles and one corner per distinct position.
- Bounds after the change of axes within 1 mm. glb is Y up, OBJ and PLY are Z up.
- PLY mesh: the mean corner colour is within ±3 of the mean texture colour at the distinct
  positions. The script samples that itself with numpy (bilinear between texel centres).

### `make_foreign.py`

It writes a scene with a fixed seed: 600 points on three planes, one colour per plane.
From it come these files:

- laspy: LAS 1.2 with 8-bit colours, LAZ 1.4 format 7 (16-bit colours, times, scan
  numbers).
- pye57: two scans with a pose each. pye57 0.4.19 cannot write a scan with spherical
  coordinates only; that case is covered by the e57-crate test in
  `crates/driftless-export/tests/e57.rs`.
- open3d: PCD ascii, binary and binary_compressed; PLY.
- trimesh: glb with a PNG texture and a node transform; OBJ with `.mtl` and PNG.

`expected.json` holds what Driftless must read from each file: the number of points or
triangles, the bounds (Z up) and the colour of the first five points (16 bit). Together the
files stay below 300 KB. They are checked in under
`crates/driftless-export/tests/fixtures/foreign/` and read by
`crates/driftless-export/tests/foreign.rs`. If that test fails, fix the reader, not the
expectation.

## Run on VM 101

```sh
DRIFTLESS_RDIR=driftless-b3 ~/.local/bin/driftless-vm sync
DRIFTLESS_RDIR=driftless-b3 ~/.local/bin/driftless-vm run cargo build --release -p driftless-cli
# then per reference, e.g.:
driftless export bezug.las -o aus.e57
~/venv-export/bin/python -I tools/export/check_export.py bezug.las aus.e57
```
