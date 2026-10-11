"""Trial parts on a plate of their own (build/print/trials, slice plate "trials"): the default design is untouched.

- The robot's drive blocks (the default variant, Layout.blocks), two fan mounts and two impellers (fan.py).
- Wheel pairs with more backlash than round 8's 0.20 (TrialParams.backlash: 0.20, 0.24, 0.28 with the brass
  pinion): the printed 36T unshifted, to suit the unshifted brass pinion (the robot's is shifted -0.45 for the
  printed pinion that is no longer made), with 0.2 mm of tip clearance each way instead of the standard 0.125 (tips
  Ø18.85, within the brass 36T's Ø19.0, which the layout clears), its teeth thinned by a backlash allowance. Design
  backlash with the brass pinion, both flanks: 0.025 + 0.5 x allowance (tools/check_gears.py measures it).
  drive.wheel engraves the value in hundredths of a mm ("24" = 0.24) on the gear's inboard face, like every wheel.
- The tire cutting guide (in build/print/alternatives): the bought tire tube (16 inside, 2 thick, 10 long) slides
  onto the base's post until it stands on the shoulder; the ring drops over it onto the base and turns round the
  shoulder. A #11 blade through the ring's slit, its tip down to the post, rides on the slit's lower edge: turn the
  ring once and the tire is cut at the channel's width (Params.tire_w) above the shoulder. Take the ring off, slide
  the cut tire off, push the rest of the tube down to the shoulder, and cut again (three tires from one tube).
"""

from dataclasses import dataclass

from build123d import Align, Cylinder, Pos, extrude

from . import drive
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass(frozen=True)
class TrialParams:
    # wheels: design backlash with the brass pinion (mm, both flanks) -> the wheel's allowance (module units)
    backlash: tuple = (0.20, 0.24, 0.28)  # round 9 (round 8's 0.20 / 0.16 / 0.12 / 0.08)
    fan_copies: int = 2
    wheel_shift: float = 0.0
    # 0.2 mm of tip clearance both ways (the standard 0.25 module leaves 0.125: a print error closing the centre
    # distance by about 0.13 would put the tips on the roots before the flanks bind)
    wheel_addendum: float = 0.85
    wheel_dedendum: float = 1.40
    # tire cutting guide
    post_fit: float = 0.0  # on the tire's inside diameter (the tube grips the post)
    shoulder_h: float = 1.5
    shoulder_gap: float = 0.4  # the shoulder's diameter under the tube's outside
    ring_fit: float = 0.3  # the ring's bore over the shoulder (diametral: it turns)
    ring_wall: float = 2.5
    ring_h: float = 9.0
    slit: float = 0.6  # tall: a #11 blade is about 0.4 thick and rides on its lower edge
    blade_t: float = 0.4
    slit_arc: float = 120.0  # deg
    base_t: float = 2.0
    floor_t: float = 1.2  # under the hollow post
    post_wall: float = 1.5


TP = TrialParams()


def allowance(backlash, tp: TrialParams = TP):
    """The wheel's backlash allowance (module units) that gives this design backlash (mm) with the brass pinion."""
    return (backlash - 0.025) / 0.5


def wheel_spec(backlash, tp: TrialParams = TP):
    """gears.spec's overrides for the trial wheel with this design backlash (mm)."""
    return {"shift": tp.wheel_shift, "backlash": allowance(backlash, tp), "addendum": tp.wheel_addendum,
            "dedendum": tp.wheel_dedendum}


def wheels(tp: TrialParams = TP, p: Params = P):
    """{wheel_bl20_L: ..., wheel_bl20_R: ..., ...}: a pair per design backlash (drive.wheel engraves the value)."""
    if p.gears.wheel_gear != "printed":
        raise ValueError("the backlash trial wheels are printed-gear wheels (Gears.wheel_gear = 'printed')")
    out = {}
    for bl in tp.backlash:
        for side in (1, -1):
            part = drive.wheel(side, p, gear=wheel_spec(bl, tp))
            part.label = f"wheel_bl{round(bl * 100):02d}_{'L' if side > 0 else 'R'}"
            out[part.label] = part
    return out


def fan_spares(tp: TrialParams = TP, p: Params = P):
    """{fan_mount_1, fan_mount_2, impeller_1, impeller_2}: the robot's fan mount and impeller, tp.fan_copies each."""
    from . import fan
    out = {}
    for k in range(1, tp.fan_copies + 1):
        for part in (fan.mount(p), fan.impeller(p)):
            part.label = f"{part.label}_{k}"
            out[part.label] = part
    return out


def tire_guide(tp: TrialParams = TP, p: Params = P):
    """(base, ring) of the tire cutting guide, base on z = 0, axis on z."""
    from build123d import Cone
    wh = p.wheel
    od = wh.tire_id + 2 * wh.tire_t
    r_sh = od / 2 + tp.shoulder_gap / 2
    r_ring = r_sh + tp.ring_fit / 2
    r_out = r_ring + tp.ring_wall
    z_sh = tp.base_t + tp.shoulder_h  # the tube stands here
    base = Cylinder(r_out + 1.0, tp.base_t, align=MIN)
    base += Pos(0, 0, tp.base_t) * Cylinder(r_sh, tp.shoulder_h, align=MIN)
    post_h = wh.tire_len + 1.0
    base += Pos(0, 0, z_sh) * Cylinder((wh.tire_id + tp.post_fit) / 2, post_h, align=MIN)
    base += Pos(0, 0, z_sh + post_h) * Cone((wh.tire_id + tp.post_fit) / 2, wh.tire_id / 2 - 1.0, 1.0, align=MIN)  # lead-in
    # hollow, open at the top (it prints post up: a cup open to the vat, not a suction cup)
    base -= Pos(0, 0, tp.floor_t) * Cylinder(wh.tire_id / 2 - 1.0 - tp.post_wall, 30, align=MIN)
    base.label, base.color = "tire_guide_base", (0.6, 0.6, 0.65)
    # the ring stands on the base's top, round the shoulder; the slit's lower edge is the cut, tire_w above the
    # shoulder's top
    ring = Cylinder(r_out, tp.ring_h, align=MIN) - Cylinder(r_ring, 2 * tp.ring_h, align=MIN)
    z_cut = tp.shoulder_h + p.tire_w - tp.blade_t / 2  # the blade's edge runs mid-thickness
    from build123d import Polygon
    from math import cos, radians, sin
    fan_pts = [(0, 0)] + [((r_out + 2) / cos(radians(tp.slit_arc / 8)) * cos(radians(a)),
                           (r_out + 2) / cos(radians(tp.slit_arc / 8)) * sin(radians(a)))
                          for a in [-tp.slit_arc / 2 + tp.slit_arc / 4 * k for k in range(5)]]
    ring -= Pos(0, 0, z_cut) * extrude(Polygon(*fan_pts, align=None), tp.slit)
    # grip ribs round the outside
    for k in range(8):
        a = 360 * k / 8 + 90
        if abs((a + 180) % 360 - 180) <= tp.slit_arc / 2 + 10:
            continue
        ring += Pos(r_out * cos(radians(a)), r_out * sin(radians(a)), 0) * Cylinder(0.8, tp.ring_h, align=MIN)
    ring = Pos(r_out * 2 + 6, 0, tp.base_t) * ring  # beside the base (for the pictures)
    ring.label, ring.color = "tire_guide_ring", (0.6, 0.6, 0.65)
    return base, ring


def parts(tp: TrialParams = TP, p: Params = P):
    """The trials plate: the robot's drive blocks (Layout.blocks), two fan mounts and impellers, and the backlash
    wheel pairs."""
    return {**drive.printed(p), **fan_spares(tp, p), **wheels(tp, p)}


def tools(tp: TrialParams = TP, p: Params = P):
    """The tire cutting guide (build/print/alternatives)."""
    return {part.label: part for part in tire_guide(tp, p)}
