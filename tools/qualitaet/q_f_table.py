"""Texture figures of surface runs side by side: q_f_table.py <label>=<dir> ...  where each dir
holds <scan>/oberflaeche/oberflaeche.{json,glb}. Prints one markdown table per scan set.
"""
import json
import os
import sys

SCANS = ["20261022133554", "20261022132520", "20261022133028", "20261020173822"]
NAMES = {"20261022133554": "S1", "20261022132520": "S2", "20261022133028": "S3", "20261020173822": "B"}


def row(d):
    o = os.path.join(d, "oberflaeche")
    try:
        j = json.load(open(os.path.join(o, "oberflaeche.json")))
    except OSError:
        return None
    s = j["stats"]
    t, m = s["texture"], s["seams"]
    faces = t["faces_textured"] + t["faces_untextured"]
    return {
        "Karten": f'{t["charts"]:,}'.replace(",", "."),
        "Inseln übernommen": f'{t.get("islands_absorbed", 0):,}'.replace(",", "."),
        "Seiten": t["pages"],
        ".glb MB": round(os.path.getsize(os.path.join(o, "oberflaeche.glb")) / 1e6),
        "seam_step Mittel / Median": f'{m["seam_step_after"]:.1f} / {m["seam_step_median_after"]:.1f}',
        "seam_jump": f'{m["seam_jump_after"]:.1f}',
        "ohne Textur %": f'{100 * t["faces_untextured"] / faces:.2f}',
        "Wände Median mm": f'{s["accuracy"]["walls"]["median_mm"]:.2f}',
        "Zeit s": round(j.get("seconds_total", 0)),
    }


def main(args):
    runs = [a.split("=", 1) for a in args]
    keys = None
    table = {}
    for label, d in runs:
        for scan in SCANS:
            r = row(os.path.join(d, scan))
            if r:
                keys = keys or list(r)
                table[(label, scan)] = r
    cols = [(l, s) for s in SCANS for l, _ in runs if (l, s) in table]
    print("| | " + " | ".join(f"{NAMES[s]} {l}" for l, s in cols) + " |")
    print("|---" * (len(cols) + 1) + "|")
    for k in keys or []:
        print(f"| {k} | " + " | ".join(str(table[c][k]) for c in cols) + " |")


if __name__ == "__main__":
    main(sys.argv[1:])
