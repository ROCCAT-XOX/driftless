"""Splits the surface error of a new mesh into the area an old mesh already covered and the
area it newly fills, with two measures per vertex:
  nn    distance to the nearest cloud point (what the program reports, grows with point spacing)
  plane distance to a plane fitted through the cloud points within 2 cm (spacing-independent)
Usage: q_fill_diag.py <old_dir> <new_dir>   (run directories with clean.las and oberflaeche/)
"""
import os
import sys

import laspy
import numpy as np
from scipy.spatial import cKDTree


def obj_vertices(path):
    vs = []
    with open(path, "rb") as f:
        for line in f:
            if line.startswith(b"v "):
                vs.append(line[2:])
    return np.loadtxt(vs, dtype=np.float64)[:, :3]


def cloud(d):
    las = laspy.read(os.path.join(d, "clean.las"))
    return np.column_stack([las.x, las.y, las.z])


def plane_dist(V, tree, P, r=0.02, kmin=6):
    out = np.full(len(V), np.nan)
    for i, idx in enumerate(tree.query_ball_point(V, r, workers=6)):
        if len(idx) < kmin:
            continue
        Q = P[idx]
        c = Q.mean(0)
        n = np.linalg.svd(Q - c, full_matrices=False)[2][2]
        out[i] = abs((V[i] - c) @ n)
    return out * 1e3


def stats(x):
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return "–"
    return f"{np.median(x):5.2f} / {np.percentile(x, 95):5.2f}  (n={len(x)})"


def main(old, new, sample=300_000):
    P = cloud(new)
    tree = cKDTree(P)
    Vo = obj_vertices(os.path.join(old, "oberflaeche", "oberflaeche.obj"))
    Vn = obj_vertices(os.path.join(new, "oberflaeche", "oberflaeche.obj"))
    rng = np.random.default_rng(0)
    Vn = Vn[rng.choice(len(Vn), min(sample, len(Vn)), replace=False)]
    Vo_s = Vo[rng.choice(len(Vo), min(sample, len(Vo)), replace=False)]
    d_old = cKDTree(Vo).query(Vn, workers=6)[0]
    covered = d_old < 0.015
    nn = tree.query(Vn, workers=6)[0] * 1e3
    pl = plane_dist(Vn, tree, P)
    print(f"{os.path.basename(new)}: neu abgedeckt {100 * (~covered).mean():.1f} % der Ecken")
    print("                     Median / 95 % (mm)")
    print("alt   nn    ", stats(tree.query(Vo_s, workers=6)[0] * 1e3))
    print("alt   plane ", stats(plane_dist(Vo_s, tree, P)))
    for name, m in (("bisher", covered), ("neu", ~covered)):
        print(f"{name:6}nn    ", stats(nn[m]))
        print(f"{name:6}plane ", stats(pl[m]))
    pts_near = np.array([len(i) for i in tree.query_ball_point(Vn[~covered][:20000], 0.02, workers=6)])
    print("neu: Punkte im 2-cm-Radius Median", np.median(pts_near) if len(pts_near) else "–",
          " Anteil <6:", f"{100 * (pts_near < 6).mean():.1f} %" if len(pts_near) else "–")


if __name__ == "__main__":
    main(*sys.argv[1:3])
