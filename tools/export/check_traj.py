#!/usr/bin/env python3
"""Checks trajectory exports (CSV, TUM) of `driftless export` against the `traj.tum` they
came from, with plain Python only (no Driftless code, no libraries).

    check_traj.py <traj.tum> <export.csv|export.tum>...

CSV, as German spreadsheets read it: `;` between the fields, a decimal comma, the header
`stempel;zeit_s;x;y;z;qx;qy;qz;qw`; `stempel` is the stamp of the pose as in TUM (bag
clock), `zeit_s` the seconds since the first pose (3 decimals). TUM: the same fields as the
source, separated by spaces. A stamp must be the same double as in the source (Driftless
reads the source as doubles and writes them with 9 decimals); the other fields must lie
within 2e-9 of it.

Per file one table row; the exit code is 1 if any file deviates.
"""

from decimal import Decimal
import sys

CSV_HEADER = "stempel;zeit_s;x;y;z;qx;qy;qz;qw"
# Positions and quaternions: the export writes `{:.9}` of the values read back; the
# quaternion is normalised when it is read, so its last digit may differ.
TOL = Decimal("2e-9")
# `zeit_s` has 3 decimals.
TOL_TIME = Decimal("0.0005")


def read_tum(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 8:
                raise ValueError(f"{path}: 8 Felder erwartet: {line!r}")
            rows.append([Decimal(v) for v in fields])
    return rows


def read_csv(path):
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    if "\r" in text:
        raise ValueError(f"{path}: CR im Text, erwartet nur \\n")
    lines = text.split("\n")
    if lines[-1] != "":
        raise ValueError(f"{path}: kein Zeilenende nach der letzten Zeile")
    lines = lines[:-1]
    if not lines or lines[0] != CSV_HEADER:
        raise ValueError(f"{path}: Kopfzeile {lines[:1]!r}, erwartet {CSV_HEADER!r}")
    rows = []
    for n, line in enumerate(lines[1:], start=2):
        fields = line.split(";")
        if len(fields) != 9:
            raise ValueError(f"{path} Zeile {n}: 9 Felder erwartet: {line!r}")
        if any("." in v for v in fields):
            raise ValueError(f"{path} Zeile {n}: Punkt statt Komma: {line!r}")
        rows.append([Decimal(v.replace(",", ".")) for v in fields])
    return rows


def check(source, path):
    """(ok, poses, largest deviation of a position or rotation, of zeit_s, note)."""
    if path.lower().endswith(".csv"):
        rows = read_csv(path)
        if len(rows) != len(source):
            return False, len(rows), None, None, f"{len(rows)} statt {len(source)} Lagen"
        t0 = float(source[0][0]) if source else 0.0
        worst, worst_time = Decimal(0), Decimal(0)
        for s, r in zip(source, rows):
            if float(r[0]) != float(s[0]):
                return False, len(rows), None, None, f"stempel {r[0]} statt {s[0]}"
            # The difference of two doubles near each other is exact.
            since = Decimal(float(s[0]) - t0)
            worst_time = max(worst_time, abs(r[1] - since))
            worst = max([worst] + [abs(a - b) for a, b in zip(r[2:], s[1:])])
        ok = worst <= TOL and worst_time <= TOL_TIME
        return ok, len(rows), worst, worst_time, ""
    rows = read_tum(path)
    if len(rows) != len(source):
        return False, len(rows), None, None, f"{len(rows)} statt {len(source)} Lagen"
    worst = Decimal(0)
    for s, r in zip(source, rows):
        if float(r[0]) != float(s[0]):
            return False, len(rows), None, None, f"Zeit {r[0]} statt {s[0]}"
        worst = max([worst] + [abs(a - b) for a, b in zip(r[1:], s[1:])])
    return worst <= TOL, len(rows), worst, None, ""


def main(argv):
    if len(argv) < 3:
        print("check_traj.py <traj.tum> <export.csv|export.tum>...", file=sys.stderr)
        return 2
    source = read_tum(argv[1])
    print("| Datei | Lagen | Abweichung Lage/Drehung | Abweichung zeit_s | Ergebnis |")
    print("|---|---|---|---|---|")
    failed = False
    for path in argv[2:]:
        try:
            ok, n, worst, worst_time, note = check(source, path)
        except (OSError, ValueError, ArithmeticError) as e:
            ok, n, worst, worst_time, note = False, "–", None, None, str(e)
        failed |= not ok
        fmt = lambda d: "–" if d is None else ("0" if d == 0 else f"{float(d):.1e}")
        verdict = "ok" if ok else f"ABWEICHUNG {note}".strip()
        print(f"| {path} | {n} | {fmt(worst)} | {fmt(worst_time)} | {verdict} |")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
