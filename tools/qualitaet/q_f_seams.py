"""Where the seam step comes from: rows of DRIFTLESS_DIAG_SEAMS (f32 × 8 per sample on an edge
between two images: step, cos of the faces, luminance change along the edge within a face,
side ratio, faces of the smaller / larger chart, image of either side).
Usage: q_f_seams.py <seams.f32>
"""
import sys

import numpy as np


def main(path):
    r = np.fromfile(path, dtype="<f4").reshape(-1, 8)
    step, cos, content, ratio, smin, smax = (r[:, k] for k in range(6))
    total = step.sum()
    q = np.percentile(step, [50, 90, 95, 99])
    print(f"{len(step)} Proben, Mittel {step.mean():.2f}, Median/p90/p95/p99 {q.round(1)}")
    print(f"Anteil am Mittel der größten 5 %: {100 * step[step >= q[2]].sum() / total:.1f} %")
    classes = [
        ("Knick > 45°", cos < np.sqrt(0.5)),
        ("Inhalt entlang der Kante > 30", (cos >= np.sqrt(0.5)) & (content > 30)),
        ("Seiten > 3× (sRGB)", (cos >= np.sqrt(0.5)) & (content <= 30) & (ratio > 3)),
        ("Rest", (cos >= np.sqrt(0.5)) & (content <= 30) & (ratio <= 3)),
    ]
    print(f"{'Klasse':32} {'Proben %':>9} {'Mittel':>7} {'Anteil Summe %':>15}")
    for name, m in classes:
        print(f"{name:32} {100 * m.mean():9.1f} {step[m].mean() if m.any() else 0:7.2f} "
              f"{100 * step[m].sum() / total:15.1f}")
    print("Kleinere Karte an der Naht (Flächen):")
    for lo, hi in [(1, 4), (4, 16), (16, 64), (64, 256), (256, 1e12)]:
        m = (smin >= lo) & (smin < hi)
        print(f"  {int(lo):>4}–{'' if hi > 1e9 else int(hi) - 1:<5} Proben {100 * m.mean():5.1f} %  "
              f"Mittel {step[m].mean() if m.any() else 0:6.2f}  Anteil {100 * step[m].sum() / total:5.1f} %")


if __name__ == "__main__":
    main(sys.argv[1])
