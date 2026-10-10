"""Trial parts on a plate of their own (build/print/trials, slice plate "trials"): the default design is untouched.

- Impellers for a fan motor with a plain Ø1.5 shaft instead of the 9T pinion: the same body (it fits the same
  mount) and 6 mm of shaft out of its front face, not cut. The radial impeller with a round blind bore whose bottom
  stops the shaft's end where the pinion put the hub (0.3 mm below the motor's front boss), so it needs no setting:
  push it on until it stops and wick in thin CA. Two bores, both tight: holes print about 0.06 small, so 1.55 comes
  out about 1.49 (a hard press) and 1.58 about 1.52 (a snug push); not under the shaft, or the 2 mm of resin round
  it splits. One dot on the backplate for the first, two for the second. The impeller prints tilted 45 deg, so the
  bore comes out a little stepped and oval: if it's too tight, ream it with a Ø1.5 drill turned by hand.
- The tire cutting guide: the bought tire tube (16 inside, 2 thick, 10 long) slides onto the base's post until it
  stands on the shoulder; the ring drops over it onto the base and turns round the shoulder. A #11 blade through
  the ring's slit, its tip down to the post, rides on the slit's lower edge: turn the ring once and the tire is cut
  at the channel's width (Params.tire_w) above the shoulder. Take the ring off, slide the cut tire off, push the rest
  of the tube down to the shoulder, and cut again (three tires from one tube).
- Wheel pairs with less backlash than the robot's (0.25 mm with the brass pinion): the printed 36T unshifted, to
  suit the unshifted brass pinion (the robot's is shifted -0.45 for the printed pinion that is no longer made; the
  tips, Ø18.85, are within the brass 36T's Ø19.0, which the layout clears; 0.2 mm of tip clearance each way
  instead of the standard 0.125), its teeth thinned by a backlash allowance. Design
  backlash with the brass pinion, both flanks: 0.025 + 0.5 x allowance (tools/check_gears.py measures it). The
  value is engraved in hundredths of a mm ("12" = 0.12) on the gear's inboard face, which prints on top (clean).
  Round 6 showed the print closes the mesh by about 0.06-0.09 mm, so the 0.08 pair may bind.
"""

from dataclasses import dataclass

from build123d import Align, Cylinder, Plane, Pos, Text, extrude

from . import drive, fan
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass(frozen=True)
class TrialParams:
    # fan motor with a plain shaft: (diameter, length out of the front face); bores to try (glued)
    shaft_d: float = 1.5
    shaft_l: float = 6.0
    shaft_bores: tuple = (1.55, 1.58)  # holes print ~0.06 small: a hard press (~1.49) and a snug push (~1.52)
    dot_d: float = 1.0  # the marks on the backplate's top, 0.3 deep
    # wheels: design backlash with the brass pinion (mm, both flanks) -> the wheel's allowance (module units)
    backlash: tuple = (0.20, 0.16, 0.12, 0.08)
    wheel_shift: float = 0.0
    # 0.2 mm of tip clearance both ways (the standard 0.25 module leaves 0.125: a print error closing the centre
    # distance by about 0.13 would put the tips on the roots before the flanks bind)
    wheel_addendum: float = 0.85
    wheel_dedendum: float = 1.40
    label_h: float = 3.0  # font size of the engraved value (the digits stand about 2.2 tall)
    label_r: float = 6.1  # its centre's distance from the axle: between the wheel's hole and the tooth roots
    engrave: float = 0.3
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


def impellers(tp: TrialParams = TP, p: Params = P):
    """{impeller_d155: ..., impeller_d158: ...}: the radial impeller on the plain shaft, one per bore."""
    from math import cos, radians, sin
    out = {}
    h = fan.heights(p)
    f = fan.F
    for n, bore in enumerate(tp.shaft_bores, 1):
        part = fan.impeller(p, shaft=(tp.shaft_d, bore, tp.shaft_l))
        # n dots on the backplate's top (it slopes: cut from above down to engrave below it)
        cx, cy = fan.centre(p)
        r_d = 8.0
        z_top = h["back_eye"] + f.back_t + (h["back_tip"] - h["back_eye"]) * (r_d - f.eye_d / 2) / (f.d2 / 2 - f.eye_d / 2)
        for k in range(n):
            a = radians(-8 + 16 * k)
            part -= Pos(cx + r_d * cos(a), cy + r_d * sin(a), z_top - tp.engrave) * Cylinder(tp.dot_d / 2, 5, align=MIN)
        part.label = f"impeller_d{round(bore * 100)}"
        out[part.label] = part
    return out


def wheels(tp: TrialParams = TP, p: Params = P):
    """{wheel_bl20_L: ..., wheel_bl20_R: ..., ...}: a pair per design backlash, the value engraved."""
    if p.gears.wheel_gear != "printed":
        raise ValueError("the backlash trial wheels are printed-gear wheels (Gears.wheel_gear = 'printed')")
    out = {}
    for bl in tp.backlash:
        tag = f"{round(bl * 100):02d}"
        for side in (1, -1):
            part = drive.wheel(side, p, gear=wheel_spec(bl, tp))
            # the gear's inboard face (|y| = gear_y), read from the robot's middle: the text's plane faces inboard
            s = 1 if side > 0 else -1
            plane = Plane(origin=(0, s * p.gear_y, p.axle_z + tp.label_r), x_dir=(s, 0, 0), z_dir=(0, -s, 0))
            digits = extrude(Text(tag, font_size=tp.label_h), 2 * tp.engrave)
            part -= plane * Pos(0, 0, -tp.engrave) * digits
            part.label = f"wheel_bl{tag}_{'L' if side > 0 else 'R'}"
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
    out = {**impellers(tp, p), **wheels(tp, p)}
    for part in tire_guide(tp, p):
        out[part.label] = part
    return out
