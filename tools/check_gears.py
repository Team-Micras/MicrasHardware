"""Check the printed gear pair (micras/gears.py): no interference through a full tooth pitch, the backlash, how
much the eccentric sleeves may close the centre distance before the teeth bind, and the tooth root stress.

Tooth root stress, ISO 6336 style: sigma = Ft / (b m) * Y_F * Y_S, with the form and stress-correction factors
read off the ISO charts for a 7-tooth pinion shifted +0.45 (Y_F ~2.9, Y_S ~1.6: the pinion is the weaker one) and
Ft from the motor's stall torque (generous: the wheel slips before a larger torque comes back through the gears).
FEA of the teeth needs a mesh too fine for this machine (tools/fea.py leaves the gears out).

Usage: uv run tools/check_gears.py [--png out.png]
"""
import argparse
import sys
from math import degrees, pi
from pathlib import Path

import numpy as np

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Axis, Pos  # noqa: E402

from micras import gears  # noqa: E402
from micras.params import P  # noqa: E402

g = P.gears
ratio = g.pinion_z / g.wheel_z
STALL = 5e-3  # N m, motor stall torque (assumed, generous for a 10 mm coreless motor)
RESIN = 35.0  # MPa, printed resin design strength (as tools/fea.py)


def pair(da=0.0, h=1.0):
    """(wheel, pinion) as thin slices, meshed at the nominal centre distance + da (pinion on -x)."""
    w = gears.spec(g.wheel_z, h)
    pn = gears.spec(g.pinion_z, h)
    pn.mesh_to(w, target_dir=np.array([-1.0, 0, 0]))
    return w.build_part(), Pos(-da, 0, 0) * pn.build_part(), np.array(pn.center[:2]) - (da, 0)


def sweep(da, steps=12):
    """Worst overlap volume and smallest flank gap over one pinion tooth pitch."""
    wheel, pin, (cx, cy) = pair(da)
    worst_v, min_gap = 0.0, 1e9
    for i in range(steps):
        t = 2 * pi / g.pinion_z * i / steps
        p_i = pin.rotate(Axis((cx, cy, 0), (0, 0, 1)), degrees(t))
        w_i = wheel.rotate(Axis.Z, -degrees(t) * ratio)
        common = p_i & w_i
        v = common.volume if common is not None else 0.0
        worst_v = max(worst_v, v)
        min_gap = min(min_gap, 0.0 if v > 1e-6 else p_i.distance_to(w_i))
    return worst_v, min_gap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", default="")
    args = ap.parse_args()
    pn, wh = gears.pinion(), gears.wheel_gear()
    print(f"pinion {g.pinion_z}T: tip d {g.tip_d(g.pinion_z):.3f}, {pn.volume:.1f} mm3 | "
          f"wheel {g.wheel_z}T: tip d {gears.spec(g.wheel_z, 1).addendum_radius * 2:.3f}, {wh.volume:.1f} mm3 | "
          f"centre distance {g.center_distance:.3f}")
    bad = 0
    for da in (0.0, -0.05, -0.08, -0.1, -0.15):
        v, gap = sweep(da)
        state = "binds" if v > 1e-4 else "free"
        print(f"centre distance {da:+.2f}: {state:5s} overlap {v:.4f} mm3, smallest flank gap {gap:.3f} mm")
        bad += da == 0.0 and v > 1e-4
    if args.png:
        import pyvista as pv
        sys.path.insert(0, Path(__file__).resolve().parent.as_posix())
        from render import mesh
        pv.OFF_SCREEN = True
        wheel, pin, _ = pair(0.0, 2.0)
        pl = pv.Plotter(off_screen=True, shape=(1, 2), window_size=(1800, 800))
        pl.subplot(0, 0)
        pl.add_mesh(mesh(wheel, 0.005), color=(0.9, 0.9, 0.85))
        pl.add_mesh(mesh(pin, 0.005), color=(0.95, 0.7, 0.3))
        pl.view_xy()
        pl.add_text("printed pair, meshed (0.5M 7T +0.45 / 36T -0.45)", font_size=10)
        pl.subplot(0, 1)
        pl.add_mesh(mesh(wheel, 0.005), color=(0.9, 0.9, 0.85))
        pl.add_mesh(mesh(pin, 0.005), color=(0.95, 0.7, 0.3))
        pl.view_xy()
        pl.camera.zoom(3.5)
        pl.camera.focal_point = (-8.6, 0, 1)
        pl.camera.position = (-8.6, 0, 60)
        pl.add_text("mesh zone", font_size=10)
        pl.screenshot(args.png)
    ft = STALL / (g.module * g.pinion_z / 2 * 1e-3)
    b = min(g.pinion_w, g.wheel_w)
    sigma = ft / (b * g.module) * 2.9 * 1.6
    print(f"tooth root stress at {STALL * 1e3:.0f} mN m: Ft {ft:.2f} N on {b} mm of face, {sigma:.1f} MPa, "
          f"safety factor {RESIN / sigma:.1f} (resin {RESIN:.0f} MPa)")
    print("OK" if not bad else "INTERFERENCE at the nominal centre distance")


if __name__ == "__main__":
    main()
