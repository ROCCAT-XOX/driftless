#!/usr/bin/env python3
"""Quality metrics for a Driftless run of the template scans (see README.md).

  q_analyze.py variant <run_dir>                 -> <run_dir>/metrics.json
  q_analyze.py cross <label_dir> <ref> <s2> <s3> -> <label_dir>/cross.json, canon.json (alignment, dims, coverage)

<run_dir> = <label_dir>/<scan>: the output folder of one scan (slam, map and surface results
side by side, plus the `/usr/bin/time -v` logs slam.log, map.log, surface.log).
Needs numpy, scipy, laspy.
"""
import json, sys, os
import numpy as np
import laspy
from scipy.spatial import cKDTree
from scipy.signal import find_peaks

rng = np.random.default_rng(0)


TIMES = {}


def load(path):
    las = laspy.read(path)
    P = np.vstack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)]).T
    if "gps_time" in las.point_format.dimension_names:
        TIMES[id(P)] = np.asarray(las.gps_time, dtype=np.float64)
    return P


def normals(S, k=16):
    tree = cKDTree(S)
    _, idx = tree.query(S, k=k, workers=6)
    X = S[idx] - S[idx].mean(axis=1, keepdims=True)
    C = np.einsum("nki,nkj->nij", X, X) / k
    w, v = np.linalg.eigh(C)
    n = v[:, :, 0]
    plan = w[:, 0] / np.maximum(w.sum(axis=1), 1e-12)
    return n, plan


def fit_plane(P):
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    return vt[2], c


def peaks_1d(vals, lo, hi, bin=0.01):
    edges = np.arange(lo, hi + bin, bin)
    h, _ = np.histogram(vals, edges)
    hs = np.convolve(h, np.ones(3) / 3, mode="same")
    pk, _ = find_peaks(hs, prominence=max(hs.max() * 0.03, 5), distance=10)
    return (edges[pk] + bin / 2), hs[pk]


def detect(P, nsub=400_000):
    """Manhattan frame and plane candidates on a subsample."""
    S = P[rng.choice(len(P), min(nsub, len(P)), replace=False)]
    n, plan = normals(S)
    good = plan < 0.01
    hor = good & (np.abs(n[:, 2]) > 0.95)
    ver = good & (np.abs(n[:, 2]) < 0.15)
    th = np.mod(np.arctan2(n[ver, 1], n[ver, 0]), np.pi / 2)
    h, e = np.histogram(th, np.linspace(0, np.pi / 2, 181))
    t0 = e[np.argmax(h)] + (e[1] - e[0]) / 2
    # circular refinement (period 90 deg)
    d = np.angle(np.exp(4j * (th - t0))) / 4
    t0 = t0 + d[np.abs(d) < np.radians(2)].mean()
    d1 = np.array([np.cos(t0), np.sin(t0), 0.0])
    d2 = np.array([-np.sin(t0), np.cos(t0), 0.0])
    cands = []
    for axis, dvec in (("x", d1), ("y", d2)):
        sel = ver & (np.abs(n @ dvec) > 0.97)
        o = S[sel] @ dvec
        if len(o) < 100:
            continue
        pos, cnt = peaks_1d(o, o.min(), o.max())
        for p, c in zip(pos, cnt):
            m = sel.copy()
            m[sel] = np.abs(o - p) < 0.03
            pts = S[m]
            if len(pts) < 200:
                continue
            other = d2 if axis == "x" else d1
            u = pts @ other
            ext_u = np.percentile(u, 97) - np.percentile(u, 3)
            ext_z = np.percentile(pts[:, 2], 97) - np.percentile(pts[:, 2], 3)
            cands.append(dict(axis=axis, n=dvec, off=float(p), count=int(len(pts)),
                              ext_u=float(ext_u), ext_z=float(ext_z),
                              u_rng=[float(np.percentile(u, 2)), float(np.percentile(u, 98))],
                              z_rng=[float(np.percentile(pts[:, 2], 2)), float(np.percentile(pts[:, 2], 98))]))
    z = S[hor, 2]
    pos, cnt = peaks_1d(z, z.min(), z.max())
    for p, c in zip(pos, cnt):
        pts = S[hor][np.abs(z - p) < 0.03]
        if len(pts) < 200:
            continue
        ext_u = np.percentile(pts @ d1, 97) - np.percentile(pts @ d1, 3)
        ext_v = np.percentile(pts @ d2, 97) - np.percentile(pts @ d2, 3)
        cands.append(dict(axis="z", n=np.array([0, 0, 1.0]), off=float(p), count=int(len(pts)),
                          ext_u=float(ext_u), ext_z=float(ext_v),
                          u_rng=[float(np.percentile(pts @ d1, 2)), float(np.percentile(pts @ d1, 98))],
                          z_rng=[float(np.percentile(pts @ d2, 2)), float(np.percentile(pts @ d2, 98))]))
    return d1, d2, cands, len(S) / len(P)


def pick_room(cands):
    """Outermost large wall per axis, lowest/highest large horizontal plane."""
    room = {}
    for axis in ("x", "y"):
        cs = [c for c in cands if c["axis"] == axis and c["ext_u"] >= 1.5 and c["ext_z"] >= 1.0]
        if not cs:
            continue
        top = max(c["count"] for c in cs)
        cs = [c for c in cs if c["count"] >= 0.15 * top]
        room[axis + "_lo"] = min(cs, key=lambda c: c["off"])
        room[axis + "_hi"] = max(cs, key=lambda c: c["off"])
    hz = [c for c in cands if c["axis"] == "z" and c["ext_u"] >= 2.0 and c["ext_z"] >= 2.0]
    if hz:
        top = max(c["count"] for c in hz)
        hz = [c for c in hz if c["count"] >= 0.15 * top]
        room["floor"] = min(hz, key=lambda c: c["off"])
        room["ceiling"] = max(hz, key=lambda c: c["off"])
        if room["ceiling"] is room["floor"]:
            del room["ceiling"]
    return room


def refine(P, cand, d1, d2, band=0.04):
    """Fit the plane on the full cloud; global and local (30x30 cm patch) residuals in mm."""
    n0 = cand["n"]
    if cand["axis"] == "z":
        a, b = d1, d2
    else:
        a = d2 if cand["axis"] == "x" else d1
        b = np.array([0, 0, 1.0])
    ua, ub = P @ a, P @ b
    m = (np.abs(P @ n0 - cand["off"]) < band) & (ua > cand["u_rng"][0]) & (ua < cand["u_rng"][1]) \
        & (ub > cand["z_rng"][0]) & (ub < cand["z_rng"][1])
    pts = P[m]
    T = TIMES.get(id(P))
    tm = T[m] if T is not None else None
    n, c = n0, pts.mean(axis=0)
    for tol in (0.03, 0.02, 0.02):
        r = (pts - c) @ n
        n, c = fit_plane(pts[np.abs(r) < tol])
        if n @ n0 < 0:
            n = -n
    r = (pts - c) @ n
    rr = r[np.abs(r) < band]
    # local patches
    pa, pb = (pts - c) @ a, (pts - c) @ b
    key = np.floor(pa / 0.3).astype(np.int64) * 100000 + np.floor(pb / 0.3).astype(np.int64)
    order = np.argsort(key)
    ks, starts = np.unique(key[order], return_index=True)
    loc_rms, loc_p95, loc_thick, pass_spread, pass_rms, npass = [], [], [], [], [], []
    for i in range(len(ks)):
        j = order[starts[i]: starts[i + 1] if i + 1 < len(ks) else len(order)]
        if len(j) < 300:
            continue
        q = pts[j]
        nl, cl = fit_plane(q)
        rl = (q - cl) @ nl
        p95 = np.percentile(np.abs(rl), 95)
        if abs(nl @ n) < np.cos(np.radians(5)) or abs(np.median((q - c) @ n)) > 0.01:  # object, not wall
            continue
        if tm is not None and np.ptp(tm[j]) > 0:
            tj = tm[j]
            o = np.argsort(tj)
            cut = np.where(np.diff(tj[o]) > 5.0)[0]
            groups = np.split(o, cut + 1)
            meds, within = [], []
            for g in groups:
                if len(g) >= 40:
                    mg = np.median(rl[g])
                    meds.append(mg)
                    within.append(rl[g] - mg)
            if len(meds) >= 2:
                pass_spread.append(max(meds) - min(meds))
                npass.append(len(meds))
            if within:
                w = np.concatenate(within)
                pass_rms.append(np.sqrt(np.mean(w ** 2)))
        loc_rms.append(np.sqrt(np.mean(rl ** 2)))
        loc_p95.append(p95)
        loc_thick.append(np.percentile(rl, 97.5) - np.percentile(rl, 2.5))
    out = dict(n=n.tolist(), c=c.tolist(), points=int(len(pts)),
               global_rms_mm=float(np.sqrt(np.mean(rr ** 2)) * 1e3),
               global_p95_mm=float(np.percentile(np.abs(rr), 95) * 1e3),
               patches=len(loc_rms))
    if loc_rms:
        out.update(local_rms_mm=float(np.median(loc_rms) * 1e3),
                   local_p95_mm=float(np.median(loc_p95) * 1e3),
                   local_thickness95_mm=float(np.median(loc_thick) * 1e3))
    if pass_rms:
        out.update(within_pass_rms_mm=float(np.median(pass_rms) * 1e3))
    if pass_spread:
        out.update(pass_offset_spread_mm=float(np.median(pass_spread) * 1e3),
                   pass_offset_spread_p90_mm=float(np.percentile(pass_spread, 90) * 1e3),
                   passes_median=float(np.median(npass)), patches_multi_pass=len(pass_spread))
    return out, pts, a, b


def room_metrics(P, d1, d2, room):
    planes, rawpts = {}, {}
    for k, cand in room.items():
        planes[k], rawpts[k], _, _ = refine(P, cand, d1, d2)
    dims = {}
    for ax in ("x", "y"):
        lo, hi = planes.get(ax + "_lo"), planes.get(ax + "_hi")
        if not (lo and hi):
            continue
        nl, nh = np.array(lo["n"]), np.array(hi["n"])
        nb = nl + nh
        nb /= np.linalg.norm(nb)
        dist = (np.array(hi["c"]) - np.array(lo["c"])) @ nb
        ang = np.degrees(np.arccos(min(1, abs(nl @ nh))))
        # separation in 3 segments along the wall (drift/bending indicator)
        lat = d2 if ax == "x" else d1
        ql, qh = rawpts[ax + "_lo"], rawpts[ax + "_hi"]
        u0 = max(np.percentile(ql @ lat, 2), np.percentile(qh @ lat, 2))
        u1 = min(np.percentile(ql @ lat, 98), np.percentile(qh @ lat, 98))
        seg = []
        for s in range(3):
            a0, a1 = u0 + (u1 - u0) * s / 3, u0 + (u1 - u0) * (s + 1) / 3
            ml = ((ql @ lat) > a0) & ((ql @ lat) < a1)
            mh = ((qh @ lat) > a0) & ((qh @ lat) < a1)
            if ml.sum() > 100 and mh.sum() > 100:
                seg.append(float((np.median(qh[mh] @ nb) - np.median(ql[ml] @ nb))))
        dims[ax] = dict(dist_m=float(dist), angle_deg=float(ang), segments_m=seg,
                        segment_spread_mm=float((max(seg) - min(seg)) * 1e3) if len(seg) > 1 else None)
    if "floor" in planes and "ceiling" in planes:
        nf, nc = np.array(planes["floor"]["n"]), np.array(planes["ceiling"]["n"])
        dims["z"] = dict(dist_m=float((np.array(planes["ceiling"]["c"]) - np.array(planes["floor"]["c"])) @ nf),
                         angle_deg=float(np.degrees(np.arccos(min(1, abs(nf @ nc))))))
    if "floor" in planes:
        nf = np.array(planes["floor"]["n"])
        dims["wall_floor_deviation_deg"] = {k: float(abs(90 - np.degrees(np.arccos(abs(np.array(p["n"]) @ nf)))))
                                            for k, p in planes.items() if k[0] in "xy"}
    return planes, dims


def all_planes_noise(P, d1, d2, cands, maxn=12):
    """Noise over the largest planar candidates (walls, floor, ceiling, big furniture)."""
    big = sorted([c for c in cands if c["ext_u"] >= 1.0 and c["ext_z"] >= 0.8], key=lambda c: -c["count"])[:maxn]
    res = []
    for c in big:
        r, _, _, _ = refine(P, c, d1, d2)
        r["axis"] = c["axis"]
        res.append(r)
    return res


def summarize(planes):
    def med(key):
        v = [p[key] for p in planes if key in p]
        return float(np.median(v)) if v else None
    return dict(n=len(planes), within_pass_rms_mm=med("within_pass_rms_mm"),
                pass_offset_spread_mm=med("pass_offset_spread_mm"), pass_offset_spread_p90_mm=med("pass_offset_spread_p90_mm"),
                local_rms_mm=med("local_rms_mm"), local_p95_mm=med("local_p95_mm"),
                local_thickness95_mm=med("local_thickness95_mm"),
                global_rms_mm=med("global_rms_mm"), global_p95_mm=med("global_p95_mm"))


def strip(p):
    return {k: v for k, v in p.items() if k not in ("n", "c")}


def time_rss(log):
    out = {}
    try:
        for line in open(log, errors="replace"):
            if "Elapsed (wall clock)" in line:
                t = line.rsplit(" ", 1)[1].strip().split(":")
                out["seconds"] = sum(float(x) * 60 ** i for i, x in enumerate(reversed(t)))
            if "Maximum resident set size" in line:
                out["max_rss_gb"] = int(line.split()[-1]) / 1048576
            if "Exit status" in line:
                out["exit"] = int(line.split()[-1])
    except FileNotFoundError:
        pass
    return out


def variant(d):
    d = os.path.abspath(d)
    out = dict(scan=os.path.basename(d), steps={})
    for st in ("slam", "map", "surface", "color"):
        tr = time_rss(os.path.join(d, st + ".log"))
        if tr:
            out["steps"][st] = tr
    sj = json.load(open(os.path.join(d, "slam.json")))
    acc = [l for l in sj["loops"] if l["accepted"]]
    rej = [l for l in sj["loops"] if not l["accepted"]]
    reasons = {}
    for l in rej:
        reasons[l["reason"]] = reasons.get(l["reason"], 0) + 1
    out["traj"] = dict(submaps=len(sj["submaps"]), loops_accepted=len(acc), loops_total=len(sj["loops"]),
                       loops_rejected_reasons=reasons,
                       loop_rms_mm_mean=float(np.mean([l["rms"] for l in acc]) * 1e3) if acc else None,
                       loop_rms_mm_max=float(np.max([l["rms"] for l in acc]) * 1e3) if acc else None,
                       loop_inlier_mean=float(np.mean([l["inlier_ratio"] for l in acc])) if acc else None,
                       start_end_gap_m=sj["start_end_gap"], path_length_m=sj["path_length"], graph=sj.get("graph"))
    # trajectory correction size (odom vs corrected)
    try:
        t1 = np.loadtxt(os.path.join(d, "traj.tum"))[:, 1:4]
        t0 = np.loadtxt(os.path.join(d, "traj_odom.tum"))[:, 1:4]
        corr = np.linalg.norm(t1 - t0, axis=1)
        out["traj"]["loop_correction_mm"] = dict(median=float(np.median(corr) * 1e3), max=float(corr.max() * 1e3))
    except Exception as e:
        out["traj"]["loop_correction_mm"] = str(e)
    mj = json.load(open(os.path.join(d, "map.json")))
    st = mj["steps"]
    out["map"] = dict(voxel=mj["settings"]["dense"]["voxel"], downscale=mj["settings"]["color"]["downscale"],
                      clean=st.get("clean", {}).get("stats"),
                      dense=st.get("dense", {}).get("stats"),
                      color={k: st.get("color", {}).get("stats", {}).get(k) for k in
                             ("points_colored", "colored_share", "images_used", "images_blurry", "seam_jump_after")},
                      gpu=st.get("color", {}).get("gpu"),
                      seconds={k: st[k].get("seconds") for k in st})
    sp = os.path.join(d, "oberflaeche", "oberflaeche.json")
    if os.path.exists(sp):
        sj2 = json.load(open(sp))
        s = sj2["stats"]
        tx = s.get("texture", {})
        tot = tx.get("faces_textured", 0) + tx.get("faces_untextured", 0)
        out["surface"] = dict(voxel=sj2["settings"]["voxel"], accuracy=s.get("accuracy"), accuracy_dense=s.get("accuracy_dense"),
                              untextured_pct=100 * tx.get("faces_untextured", 0) / tot if tot else None,
                              charts=tx.get("charts"), pages=tx.get("pages"), fill_ratio=tx.get("fill_ratio"),
                              texel_median_mm=tx.get("texel_median_mm"),
                              seam_step_after=s.get("seams", {}).get("seam_step_after"),
                              seam_step_median_after=s.get("seams", {}).get("seam_step_median_after"),
                              faces_final=s.get("faces_final"), cleanup=s.get("cleanup"),
                              glb_mb=os.path.getsize(os.path.join(d, "oberflaeche", "oberflaeche.glb")) / 1e6,
                              seconds_total=sj2.get("seconds_total"), peak_rss_gb=sj2.get("peak_rss_bytes", 0) / 1e9,
                              gpu=s.get("gpu") or sj2.get("gpu"))
    for cloud in ("clean", "dense"):
        P = load(os.path.join(d, cloud + ".las"))
        d1, d2, cands, _ = detect(P)
        room = pick_room(cands)
        planes, dims = room_metrics(P, d1, d2, room)
        noise = all_planes_noise(P, d1, d2, cands)
        out[cloud] = dict(points=int(len(P)), frame=dict(d1=d1.tolist(), d2=d2.tolist()),
                          room_planes={k: strip(p) for k, p in planes.items()},
                          room_plane_fits={k: dict(n=p["n"], c=p["c"]) for k, p in planes.items()},
                          dims=dims, planes_noise=summarize(noise),
                          planes=[strip(p) | {"axis": p["axis"]} for p in noise])
        del P
    json.dump(out, open(os.path.join(d, "metrics.json"), "w"), indent=1)
    print(json.dumps({k: out[k] for k in ("traj",)}, indent=1)[:1500])
    for cloud in ("clean", "dense"):
        print(cloud, out[cloud]["points"], json.dumps(out[cloud]["dims"]), json.dumps(out[cloud]["planes_noise"]))


# ---------------- cross-scan ----------------

def canon(metrics, cloud="clean"):
    """Rigid transform world->room frame: x along the longer room axis, origin at lo/lo/floor."""
    m = metrics[cloud]
    fits = m["room_plane_fits"]
    d1, d2 = np.array(m["frame"]["d1"]), np.array(m["frame"]["d2"])
    lx = m["dims"]["x"]["dist_m"]
    ly = m["dims"]["y"]["dist_m"]
    ox = np.array(fits["x_lo"]["c"]) @ d1
    oy = np.array(fits["y_lo"]["c"]) @ d2
    oz = np.array(fits["floor"]["c"])[2]
    if lx >= ly:
        R = np.vstack([d1, d2, [0, 0, 1]])
        t = -np.array([ox, oy, oz])
    else:
        R = np.vstack([d2, -d1, [0, 0, 1]])
        hx = np.array(fits["x_hi"]["c"]) @ d1
        t = -np.array([oy, -hx, oz])
    return R, t


def voxset(P, s):
    return set(map(tuple, np.unique(np.floor(P / s).astype(np.int32), axis=0)))


def icp(src, dst, iters=30, trim=0.1):
    tree = cKDTree(dst)
    R, t = np.eye(3), np.zeros(3)
    for i in range(iters):
        X = src @ R.T + t
        dd, j = tree.query(X, workers=6)
        m = dd < trim
        A, B = X[m], dst[j[m]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:
            Vt[2] *= -1
            Ri = Vt.T @ U.T
        ti = cb - Ri @ ca
        R, t = Ri @ R, Ri @ t + ti
        trim = max(0.03, trim * 0.85)
    X = src @ R.T + t
    dd, _ = tree.query(X, workers=6)
    return R, t, dd


def downs(P, s):
    k = np.floor(P / s).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return P[i]


def cross(base, scans):
    ms = [json.load(open(os.path.join(base, s, "metrics.json"))) for s in scans]
    clouds, mats = [], []
    def H(R, t):
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
    for s, m in zip(scans, ms):
        P = downs(load(os.path.join(base, s, "clean.las")), 0.02)
        R, t = canon(m)
        clouds.append(P @ R.T + t)
        mats.append(H(R, t))
    ref = clouds[0]
    out = dict(scans=scans, dims={}, align={}, coverage={})
    for ax in ("x", "y", "z"):
        vals = [m["clean"]["dims"].get(ax, {}).get("dist_m") for m in ms]
        valsd = [m["dense"]["dims"].get(ax, {}).get("dist_m") for m in ms]
        out["dims"][ax] = dict(clean=vals, dense=valsd,
                               spread_mm=float((max(vals) - min(vals)) * 1e3) if None not in vals else None,
                               spread_dense_mm=float((max(valsd) - min(valsd)) * 1e3) if None not in valsd else None)
    # for x/y: map lo/hi naming via canon (longer axis = x)
    for i in range(1, len(scans)):
        best = None
        for flip in (False, True):
            C = clouds[i].copy()
            if flip:  # 180 deg about z inside the room box
                L = np.array([ms[i]["clean"]["dims"]["x"]["dist_m"], ms[i]["clean"]["dims"]["y"]["dist_m"]])
                Lx, Ly = (L if L[0] >= L[1] else L[::-1])
                C[:, 0], C[:, 1] = Lx - C[:, 0], Ly - C[:, 1]
            R, t, dd = icp(C[rng.choice(len(C), min(200000, len(C)), replace=False)], ref, iters=25, trim=0.3)
            score = np.mean(dd < 0.05)
            if best is None or score > best[0]:
                best = (score, flip, C, R, t)
        score, flip, C, R, t = best
        R2, t2, dd = icp(C, ref, iters=20, trim=0.05)
        R, t = R2 @ R, R2 @ t + t2
        C = C @ R.T + t
        clouds[i] = C
        if flip:
            L = np.array([ms[i]["clean"]["dims"]["x"]["dist_m"], ms[i]["clean"]["dims"]["y"]["dist_m"]])
            Lx, Ly = (L if L[0] >= L[1] else L[::-1])
            F = H(np.diag([-1.0, -1.0, 1.0]), np.array([Lx, Ly, 0.0]))
        else:
            F = np.eye(4)
        mats[i] = H(R, t) @ F @ mats[i]
        dd_ref, _ = cKDTree(C).query(ref, workers=6)
        out["align"][scans[i]] = dict(flip=bool(flip), inlier5cm=float(np.mean(dd < 0.05)),
                                      nn_median_mm=float(np.median(dd[dd < 0.1]) * 1e3),
                                      nn_p90_mm=float(np.percentile(dd[dd < 0.1], 90) * 1e3),
                                      ref_to_scan_median_mm=float(np.median(dd_ref[dd_ref < 0.1]) * 1e3),
                                      correction_rot_deg=float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))),
                                      correction_t_mm=(t * 1e3).tolist())
    # coverage at 5 cm with 1-voxel tolerance; inside the room box only
    s = 0.05
    boxes = [c[(c[:, 2] > -0.1) & (c[:, 2] < 4.0)] for c in clouds]
    sets = [voxset(c, s) for c in boxes]
    U = set().union(*sets)
    nb = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    def dil(S):
        D = set()
        for (x, y, z) in S:
            for a, b, c in nb:
                D.add((x + a, y + b, z + c))
        return D
    dils = [dil(S) for S in sets]
    for i, sname in enumerate(scans):
        missing = [k for k in U if k not in dils[i]]
        mz = np.array([k[2] for k in missing]) * s if missing else np.zeros(0)
        out["coverage"][sname] = dict(voxels=len(sets[i]), union=len(U),
                                      missing_vs_union_pct=100 * len(missing) / len(U),
                                      missing_m2_approx=len(missing) * s * s,
                                      missing_by_height_pct={f"{a}-{a+1}m": float(100 * np.mean((mz >= a) & (mz < a + 1))) if len(mz) else 0
                                                             for a in range(0, 4)})
    # pairwise only-one-scan voxels
    only = {}
    for i, sname in enumerate(scans):
        others = set().union(*[dils[j] for j in range(len(scans)) if j != i])
        only[sname] = 100 * sum(1 for k in sets[i] if k not in others) / len(U)
    out["coverage"]["only_in_this_scan_pct_of_union"] = only
    json.dump(out, open(os.path.join(base, "cross.json"), "w"), indent=1)
    # transforms for views: world_scan -> room/ref frame
    tf = {}
    to_ref_world = np.linalg.inv(mats[0])
    for i, sname in enumerate(scans):
        # 4x4: scan world -> reference scan world (ref = first scan)
        tf[sname] = (to_ref_world @ mats[i]).tolist()
    json.dump(tf, open(os.path.join(base, "canon.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    if sys.argv[1] == "variant":
        variant(sys.argv[2])
    else:
        cross(os.path.abspath(sys.argv[2]), sys.argv[3:])
