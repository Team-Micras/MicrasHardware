"""Drill guide for the bought 36T brass gear (Gears.wheel_gear = "brass"): its bore opened up to the wheel's hole
(Params.wheel_hole_d, clear of the bearing tube) and the three M2 holes on drive.gear_screw_r, which screw it to
the printed wheel.

Two resin parts, square outside so the stack clamps in a vise. The base holds the gear in a pocket with the gear's
outline (its teeth, 0.1 mm clear all round): that centres the gear and keeps it from turning. The square lid drops
into a square recess above the pocket, so it can't turn either (the three holes must stay where the first one was
drilled), and rests on the gear (the pocket is 0.1 mm shallower than the gear), so clamping the lid clamps the gear.
The lid's holes guide the drills: the centre one the final Ø7.8 drill (open the bore up to it in steps; each drill
follows the hole before it), the three small ones a Ø2.2. Under the gear the base has a hole under each, for the
drills to come out into.

Then countersink the three holes 90 deg on one face to Ø4.0 (an M2 flat head sits flush or just below): that face
goes inboard, 0.25 mm from the block, so nothing may stand proud of it.
"""

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Pos, extrude, offset

from . import drive
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass(frozen=True)
class GuideParams:
    gear_gap: float = 0.1  # all round the gear's outline in the pocket (resin holes print about 0.03 small a side)
    lid_fit: float = 0.2  # the lid's side in the recess (0.1 a side)
    drill_fit: float = 0.1  # diametral over the drill in the lid's holes: the drill turns freely, without play
    lid_w: float = 22.0  # square
    lid_t: float = 7.0  # the drills' guided length
    recess: float = 3.0  # how deep the lid drops in (it stands 4 mm proud: hold it down by it)
    rim: float = 3.0  # wall round the recess (the vise's jaws grip it)
    floor: float = 3.0  # under the gear
    small_drill: float = 2.2  # the screws' clearance (DriveParams.screw_clear_d)
    exit_gap: float = 0.6  # the holes under the gear are this much wider than the drills


GP = GuideParams()


def _depth(p: Params):
    return p.gears.wheel_w - 0.1  # the gear stands 0.1 proud of the pocket: the lid clamps it


def _gear_outline(p: Params):
    """The brass 36T's outline (its teeth, no bore) as a face on z = 0."""
    from build123d import Axis
    from .gears import brass_spec
    g = p.gears
    return brass_spec(g.wheel_z, 1.0, p).build_part().faces().sort_by(Axis.Z)[0]


def base(gp: GuideParams = GP, p: Params = P):
    side = gp.lid_w + gp.lid_fit + 2 * gp.rim
    z_gear = gp.floor
    z_lid = z_gear + _depth(p)
    part = Box(side, side, z_lid + gp.recess, align=MIN)
    part -= Pos(0, 0, z_gear) * extrude(offset(_gear_outline(p), gp.gear_gap), 10, dir=(0, 0, 1))
    part -= Pos(0, 0, z_lid) * Box(gp.lid_w + gp.lid_fit, gp.lid_w + gp.lid_fit, 10, align=MIN)
    # where the drills come out
    part -= Cylinder((p.wheel_hole_d + gp.exit_gap) / 2, 20, align=MIN)
    for x, z, _ in drive.gear_screws(p):
        part -= Pos(x, z - p.axle_z, 0) * Cylinder((gp.small_drill + gp.exit_gap) / 2, 20, align=MIN)
    part.label, part.color = "drill_guide_base", (0.6, 0.6, 0.65)
    return part


def lid(gp: GuideParams = GP, p: Params = P):
    part = Box(gp.lid_w, gp.lid_w, gp.lid_t, align=MIN)
    part -= Cylinder((p.wheel_hole_d + gp.drill_fit) / 2, 2 * gp.lid_t, align=MIN)
    for x, z, _ in drive.gear_screws(p):
        part -= Pos(x, z - p.axle_z, 0) * Cylinder((gp.small_drill + gp.drill_fit) / 2, 2 * gp.lid_t, align=MIN)
    part.label, part.color = "drill_guide_lid", (0.6, 0.6, 0.65)
    return part


def drilled_gear(gp: GuideParams = GP, p: Params = P):
    """The brass 36T as it comes out, sitting in the base's pocket (for the pictures)."""
    from build123d import Rot
    from .gears import brass_spec
    g = p.gears
    gear = brass_spec(g.wheel_z, g.wheel_w, p).build_part()
    gear -= Cylinder(p.wheel_hole_d / 2, 10, align=MIN)
    for x, z, _ in drive.gear_screws(p):
        gear -= Pos(x, z - p.axle_z, g.wheel_w) * drive.countersunk(drive.D, depth=5, up=1)
    gear = Pos(0, 0, gp.floor) * gear
    gear.label, gear.color = "brass_gear", (0.85, 0.7, 0.3)
    return gear


def exploded(gp: GuideParams = GP, p: Params = P, lift=12.0):
    """{label: shape}: base, the drilled gear in its pocket, the lid lifted off (for the pictures)."""
    lid_ = lid(gp, p)
    return {"drill_guide_base": base(gp, p), "brass_gear": drilled_gear(gp, p),
            "drill_guide_lid": Pos(0, 0, gp.floor + _depth(p) + lift) * lid_}


def parts(gp: GuideParams = GP, p: Params = P):
    return {part.label: part for part in (base(gp, p), lid(gp, p))}
