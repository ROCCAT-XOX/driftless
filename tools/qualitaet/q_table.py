#!/usr/bin/env python3
"""Markdown table of one run: one column per scan, from <label_dir>/<scan>/metrics.json.

  q_table.py <label_dir> [scan ...]     (default: the scans found in <label_dir>)

Rows = the goal table of the quality round (Z2-Z6; Z1 "Löcher von oben" needs the view tool) plus
the run figures of the program itself (`dense_quality` in map.json, `accuracy_dense` in
oberflaeche.json) next to the Python values of the same definition.
"""
import json, os, sys

if len(sys.argv) < 2:
    sys.exit(__doc__)
BASE = os.path.abspath(sys.argv[1])
SCANS = sys.argv[2:] or sorted(d for d in os.listdir(BASE) if os.path.exists(os.path.join(BASE, d, "metrics.json")))


def g(d, *ks):
    for k in ks:
        if d is None:
            return None
        d = d.get(k) if isinstance(d, dict) else None
    return d


def f(v, nd=1):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def holes(scan):
    """LOECHER lines of <scan>/images.log: {view: percent}."""
    out = {}
    try:
        for line in open(os.path.join(BASE, scan, "images.log"), errors="replace"):
            p = line.split()
            if len(p) == 3 and p[0] == "LOECHER":
                out[p[1]] = float(p[2])
    except FileNotFoundError:
        pass
    return out


ROWS = [
    ("Löcher von oben (%)", lambda m: holes(m["scan"]).get("oben"), 1),
    ("Löcher Rundblick min–max (%)", lambda m: (lambda h: f"{min(h):.1f}–{max(h):.1f}" if h else None)([v for k, v in holes(m["scan"]).items() if k.startswith("rundblick_")]), 0),
    ("Zeit SLAM (s)", lambda m: g(m, "steps", "slam", "seconds"), 1),
    ("Zeit Karte (s)", lambda m: g(m, "steps", "map", "seconds") or g(m, "steps", "color", "seconds"), 0),
    ("Zeit Oberfläche (s)", lambda m: g(m, "steps", "surface", "seconds"), 0),
    ("RAM Karte (GB)", lambda m: g(m, "steps", "map", "max_rss_gb") or g(m, "steps", "color", "max_rss_gb"), 2),
    ("RAM Oberfläche (GB)", lambda m: g(m, "steps", "surface", "max_rss_gb"), 2),
    ("Teilkarten", lambda m: g(m, "traj", "submaps"), 0),
    ("Schleifen akzeptiert / geprüft", lambda m: f"{g(m,'traj','loops_accepted')} / {g(m,'traj','loops_total')}", 0),
    ("Schleifen-ICP RMS Mittel / max (mm)", lambda m: f"{f(g(m,'traj','loop_rms_mm_mean'))} / {f(g(m,'traj','loop_rms_mm_max'))}", 0),
    ("Start↔Ende vorher → nachher (mm)", lambda m: f"{f(1e3*g(m,'traj','start_end_gap_m','before'))} → {f(1e3*g(m,'traj','start_end_gap_m','after'))}", 0),
    ("Weglänge (m)", lambda m: g(m, "traj", "path_length_m", "after"), 1),
    ("Korrektur Schleifenschluss Median / max (mm)", lambda m: f"{f(g(m,'traj','loop_correction_mm','median'))} / {f(g(m,'traj','loop_correction_mm','max'))}", 0),
    ("Punkte dense / clean (Mio.)", lambda m: f"{g(m,'dense','points')/1e6:.1f} / {g(m,'clean','points')/1e6:.1f}", 0),
    ("dense: Rauschen je Durchgang RMS (mm)", lambda m: g(m, "dense", "planes_noise", "within_pass_rms_mm"), 1),
    ("dense: Versatz zwischen Durchgängen Median / p90 (mm)", lambda m: f"{f(g(m,'dense','planes_noise','pass_offset_spread_mm'))} / {f(g(m,'dense','planes_noise','pass_offset_spread_p90_mm'))}", 0),
    ("Programm: dense Rauschen je Durchgang RMS (mm)", lambda m: g(m, "map", "clean", "dense_quality", "within_pass_rms_mm"), 1),
    ("Programm: dense Versatz zwischen Durchgängen Median / p90 (mm)", lambda m: f"{f(g(m,'map','clean','dense_quality','pass_offset_median_mm'))} / {f(g(m,'map','clean','dense_quality','pass_offset_p90_mm'))}", 0),
    ("Programm: dense Ebene lokal RMS (mm)", lambda m: g(m, "map", "clean", "dense_quality", "local_rms_mm"), 1),
    ("Programm: dense Wanddicke 95 % (mm)", lambda m: g(m, "map", "clean", "dense_quality", "thickness95_mm"), 1),
    ("Programm: Flecken gesamt / mit mehreren Durchgängen", lambda m: f"{f(g(m,'map','clean','dense_quality','patches'))} / {f(g(m,'map','clean','dense_quality','patches_multi_pass'))}", 0),
    ("dense: Ebene lokal RMS / p95 (mm)", lambda m: f"{f(g(m,'dense','planes_noise','local_rms_mm'))} / {f(g(m,'dense','planes_noise','local_p95_mm'))}", 0),
    ("dense: Wanddicke 95 % (mm)", lambda m: g(m, "dense", "planes_noise", "local_thickness95_mm"), 1),
    ("dense: Ebene global RMS / p95 (mm)", lambda m: f"{f(g(m,'dense','planes_noise','global_rms_mm'))} / {f(g(m,'dense','planes_noise','global_p95_mm'))}", 0),
    ("clean: Ebene lokal RMS / p95 (mm)", lambda m: f"{f(g(m,'clean','planes_noise','local_rms_mm'))} / {f(g(m,'clean','planes_noise','local_p95_mm'))}", 0),
    ("clean: Ebene global RMS / p95 (mm)", lambda m: f"{f(g(m,'clean','planes_noise','global_rms_mm'))} / {f(g(m,'clean','planes_noise','global_p95_mm'))}", 0),
    ("Raum Länge × Breite × Höhe (m, clean)", lambda m: " × ".join(f(g(m, "clean", "dims", a, "dist_m"), 3) for a in "xyz"), 0),
    ("Wandabstand in 3 Abschnitten, Spanne x / y (mm)", lambda m: f"{f(g(m,'clean','dims','x','segment_spread_mm'))} / {f(g(m,'clean','dims','y','segment_spread_mm'))}", 0),
    ("Gegenwände nicht parallel x / y (°)", lambda m: f"{f(g(m,'clean','dims','x','angle_deg'),2)} / {f(g(m,'clean','dims','y','angle_deg'),2)}", 0),
    ("Oberfläche Wände Median / Mittel / 95 % (mm)", lambda m: " / ".join(f(g(m, "surface", "accuracy", "walls", k), 2) for k in ("median_mm", "mean_mm", "p95_mm")), 0),
    ("Oberfläche Kanten Median / Mittel / 95 % (mm)", lambda m: " / ".join(f(g(m, "surface", "accuracy", "edges", k), 2) for k in ("median_mm", "mean_mm", "p95_mm")), 0),
    ("Oberfläche gegen dense Wände Median / Mittel / 95 % (mm)", lambda m: " / ".join(f(g(m, "surface", "accuracy_dense", "walls", k), 2) for k in ("median_mm", "mean_mm", "p95_mm")), 0),
    ("Oberfläche gegen dense Kanten Median / Mittel / 95 % (mm)", lambda m: " / ".join(f(g(m, "surface", "accuracy_dense", "edges", k), 2) for k in ("median_mm", "mean_mm", "p95_mm")), 0),
    ("Dreiecke fertig (Mio.)", lambda m: (g(m, "surface", "faces_final") or 0) / 1e6, 2),
    ("ohne Textur (%)", lambda m: g(m, "surface", "untextured_pct"), 2),
    ("seam_step nachher Mittel / Median", lambda m: f"{f(g(m,'surface','seam_step_after'))} / {f(g(m,'surface','seam_step_median_after'))}", 0),
    ("Karten (charts)", lambda m: g(m, "surface", "charts"), 0),
    ("Füllung Seiten (%)", lambda m: 100 * g(m, "surface", "fill_ratio") if g(m, "surface", "fill_ratio") is not None else None, 1),
    ("Seiten", lambda m: g(m, "surface", "pages"), 0),
    ("Texel Median (mm)", lambda m: g(m, "surface", "texel_median_mm"), 2),
    (".glb (MB)", lambda m: g(m, "surface", "glb_mb"), 0),
]

ms = {}
for s in SCANS:
    p = os.path.join(BASE, s, "metrics.json")
    if os.path.exists(p):
        ms[s] = json.load(open(p))
if not ms:
    sys.exit(f"keine metrics.json unter {BASE}")
print(f"### {os.path.basename(BASE)}\n")
print("| Messgröße | " + " | ".join(ms) + " |")
print("|---|" + "---|" * len(ms))
for name, fn, nd in ROWS:
    cells = []
    for m in ms.values():
        try:
            cells.append(f(fn(m), nd))
        except Exception:
            cells.append("–")
    print(f"| {name} | " + " | ".join(cells) + " |")

# Z5: spread of the clean room dimensions over the black-room scans (as `q_analyze.py cross` reports)
black = [m for s, m in ms.items() if s.startswith("2026102213")]
if len(black) > 1:
    # Horizontal axes can be swapped between scans (near-square room): sort short/long per scan
    dims = []
    for m in black:
        d = [g(m, "clean", "dims", ax, "dist_m") for ax in "xyz"]
        dims.append(None if None in d else (*sorted(d[:2]), d[2]))
    sp = []
    for i in range(3):
        v = [d[i] for d in dims if d is not None]
        sp.append(f((max(v) - min(v)) * 1e3) if v and len(v) == len(dims) else "–")
    print(f"\nRaummaße Spanne der {len(black)} Schwarzraum-Scans kurz / lang / Höhe (mm, clean): " + " / ".join(sp))
