"""Exposure test for the Photon Mono 4: eight exposure coupons (fit_test.exposure_coupon) in one print, using
Anycubic's Resin Exposure Range Finder. A file named exactly R_E_R_F.pm4n makes the printer split the screen into
8 zones and expose zone k for BASE + (k - 1) x 0.25 s (from the file's exposure, BASE here).

The zones' layout isn't documented for the Mono 4 (8 strips across the long side, or 4 columns x 2 rows): each
coupon sits inside one zone either way, under one strip's centre, alternating between the two halves of the
plate. Coupon k carries the number k: its zone order is a guess (confirm it against Anycubic's own R_E_R_F file
from the printer's USB stick before trusting the numbers; the hole fits also rise with the exposure).

Output: build/sliced/R_E_R_F.pm4n (copy it to the stick under exactly that name) and build/sliced/rerf.3mf.

Usage: tools/capped.sh uv run tools/rerf.py [--base 2.0] [--copy-to E:]
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, ROOT.as_posix())
sys.path.insert(0, (ROOT / "tools").as_posix())
from build123d import export_stl  # noqa: E402

import slice as sl  # noqa: E402
from micras import fit_test  # noqa: E402

EDGE = 4.6  # mm, coupons from the screen's edges (the pad reaches 3.4 out with its border upright)
BASE = 2.0  # s: zones 2.0 .. 3.75 (Anycubic's table says 3.0, its blog 2.8, its resin manual 2.4)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--base", type=float, default=BASE, help="exposure of zone 1, s (each next zone +0.25)")
    ap.add_argument("--copy-to", default="", help="drive letter (E:) or directory to copy R_E_R_F.pm4n to")
    args = ap.parse_args()
    work = sl.OUT / "rerf"
    work.mkdir(parents=True, exist_ok=True)
    bx, by = sl.RESIN_BED[:2]
    strip = bx / 8
    plate = []
    for k in range(1, 9):
        path = work / f"exposure_{k}.stl"
        export_stl(fit_test.exposure_coupon(k), str(path), tolerance=0.01, angular_tolerance=0.1)
        v, t = sl.load(path)
        v = v - [*(v.min(0)[:2]), 0]
        w, d = v.max(0)[:2]
        # zone k: strips counted from the right (k = 1 at the top right), rows alternating
        i = 8 - k
        top = (k % 2) == 1
        # centred in its strip, but at least EDGE from the screen's edges (the pad's brim and slope reach out)
        x0 = min(max((i + 0.5) * strip - w / 2, i * strip + 0.5, EDGE), (i + 1) * strip - 0.5 - w, bx - EDGE - w)
        y0 = max(by / 2 + 1.0, by - EDGE - d) if top else min(by / 2 - 1.0 - d, EDGE)
        plate.append((f"exposure_{k}", v, t, (x0, y0)))
    src = sl.OUT / "rerf.3mf"
    sl.write_3mf(plate, src)
    ini = work / "rerf.ini"
    ini.write_text(f"exposure_time = {args.base}\npad_brim_size = 2\npad_wall_slope = 90\n")
    profiles = ["mono4.ini", f"resin_{sl.RESIN}.ini", str(ini)]
    pm4n, info, warn = sl.slice_resin("R_E_R_F", profiles, src)
    print(f"{pm4n.name}: {info}; zones {args.base:g} .. {args.base + 7 * 0.25:g} s")
    for w in warn:
        print("   ", w.strip())
    if args.copy_to:
        sl.copy_to([pm4n], args.copy_to)


if __name__ == "__main__":
    main()
