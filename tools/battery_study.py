"""Compare battery arrangements: tray position for CoM over the axle, height, CoM height, yaw inertia.

Usage: uv run tools/battery_study.py [target_com_x]
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from micras import drive, fan, layout, mass  # noqa: E402
from micras.params import P  # noqa: E402

TARGET_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
TRAY_FLOOR = 1.0  # tray floor thickness
CLEAR = 0.5
WALL = 1.0
printed = {**drive.all_parts(), **{k: v for k, v in fan.parts().items() if k != "fan_motor"}}
obstacles = []  # (x0, x1, top_z)
for name, part in {**layout.reference(with_board=False), **printed, "fan_motor": fan.parts()["fan_motor"]}.items():
    if name.startswith("cell"):
        continue
    bb = part.bounding_box()
    obstacles.append((bb.min.X, bb.max.X, bb.max.Z))
for b in layout.board_boxes():
    if abs(b["min"][1] + b["max"][1]) / 2 < 26:
        obstacles.append((b["min"][0], b["max"][0], b["max"][2]))


def floor_at(x, sx):
    x0, x1 = x - sx / 2 - WALL, x + sx / 2 + WALL
    tops = [t for a, b, t in obstacles if a < x1 and b > x0]
    return max(tops + [P.board.top_z]) + CLEAR + TRAY_FLOOR


from micras import assembly  # noqa: E402
real = {k: v for k, v in assembly.printed().items() if not k.startswith(("body", "lid"))}  # box moves with the pack
fixed = mass.fixed_items(P, real, placeholders=False, material=lambda n: assembly.material(n)[0])
base = mass.summarize(fixed)
print(f"without battery: {base['mass']:.1f} g, CoM x={base['com'][0]:+.2f} z={base['com'][2]:.1f}")
print(f"target CoM x = {TARGET_X:+.1f} mm\n")
print(f"{'arrangement':10s} {'pack x*z':>11s} {'tray x':>7s} {'floor z':>7s} {'top z':>6s} "
      f"{'CoM x':>6s} {'CoM z':>6s} {'Izz g*mm2':>10s}")
results = []
for arr in ("pyramid", "edge", "flat", "stack", "standing"):
    _, (sx, sz) = mass.battery_cells(arr, P)
    best = None
    for x in np.arange(-35, 20, 0.25):
        fz = floor_at(x, sx)
        tray = mass.Item("tray", mass.MASSES["tray"], (x, 0, fz + sz / 2))
        s = mass.summarize(fixed + mass.battery_items(arr, x, fz, P) + [tray])
        err = abs(s["com"][0] - TARGET_X)
        if best is None or err < best[0]:
            best = (err, x, fz, s)
    err, x, fz, s = best
    results.append((s["izz"], arr))
    print(f"{arr:10s} {sx:5.1f}x{sz:4.1f} {x:7.2f} {fz:7.1f} {fz + sz + 1:6.1f} "
          f"{s['com'][0]:+6.2f} {s['com'][2]:6.1f} {s['izz']:10.0f}" + ("  (CoM target missed)" if err > 0.5 else ""))
print("\nby yaw inertia:", ", ".join(f"{a} {i:.0f}" for i, a in sorted(results)))
