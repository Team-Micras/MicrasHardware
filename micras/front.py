"""Front: wall-sensor caps (black resin) and the front bumper (TPU).

Each wall sensor is a stacked pair of THT parts with bent legs: an SFH 4550 emitter (5 mm epoxy, half angle
3 deg) above a TPS601A receiver (TO-18 can with a lens and a key tab, 10 deg); leds.py models both from their
datasheets. With beams that narrow, the aim matters far more than any beam shaping
(a 1.5 deg pitch error changes the reading by 10-40 % up close), and an aperture in front of the lenses
only cuts the signal. So the cap:

- stands on the board, on the outline the footprint draws for a sensor casing, and so sets the height,
  pitch and roll of both LEDs from the board;
- holds each LED at two lands (flange and body) with small crush ribs, which absorb the print tolerance;
- locates sideways and in yaw with a fork around the four soldered legs, and axially on the flanges;
- keys the receiver's roll with a keyway for its flange tab (it runs the sleeve's length, so the cap slides
  over the tab);
- wraps the LED bodies and puts a septum between them (side light and direct crosstalk), with the front
  open at full lens width and a short hood;
- can pitch the emitter down towards the receiver (emitter_tilt) to move the reading's peak closer.

It slides on from the front along the look direction; a drop of glue on the base keeps it down.
"""

from dataclasses import dataclass
from math import atan2, degrees

from build123d import (Align, Box, Circle, Compound, Cylinder, Plane, Polyline, Pos, Rectangle, Rot, extrude, make_face,
                       make_hull, offset)

from . import layout
from .leds import EMITTER, LEG_V, RECEIVER, RECEIVER_TAB, Seat
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

# Wall sensors: footprint origin (x, y) in the robot frame and look direction (deg from +x), from the
# KiCad footprints W1..W4. Local frame: u along the look direction, v to its left, z above the board top.
SENSORS = {
    "W1": ((36.7198, 21.5), 0.0),
    "W2": ((47.5698, 10.1858), 45.0),
    "W3": ((47.5700, -10.1860), -45.0),
    "W4": ((36.7198, -21.5), 0.0),
}


LEG_W = max(EMITTER.leg_w_max, RECEIVER.leg_w_max)  # the thickest lead (the SFH's, 0.5 square, 0.6 max)
# casing outline drawn by the footprint (silkscreen and User.13), (u, v): the cap may stand on it
OUTLINE = ((7.62, 3.429), (-2.032, 3.429), (-2.032, 2.413), (-6.731, 2.413),
           (-6.731, -2.413), (-2.032, -2.413), (-2.032, -3.429), (7.62, -3.429))


@dataclass(frozen=True)
class FrontParams:
    fit: float = 0.15  # radial clearance of the bores (the ribs grip)
    rib_w: float = 0.5
    rib_interf: float = 0.1  # crush-rib interference on the LED (resin prints ~0.05 noisy: tune)
    wall: float = 0.6  # around the flange bore
    hood: float = 1.0  # beyond the receiver's lens tip (the front stays open at full lens width)
    emitter_tilt: float = 0.0  # deg, emitter pitched down towards the receiver (bench test 0, 1, 2)
    base_margin: float = 0.08  # base inside the outline
    raise_z: float = 1.45  # the cap outside the outline stays this far above the board (parts up to 1.1)
    under_slot: float = 2.0  # half width of the opening under the receiver (its flange is 0.45 above the board)
    fork_z: tuple = (1.35, 2.15)  # tines: clear of the parts behind, below the receiver's leg bend (2.58)
    fork_tail: float = 0.4  # behind the emitter's legs
    slot_w: float = 0.8  # around the legs (0.6 max; bent by hand: +-0.1; resin closes slots a little)
    slot_lead: float = 0.6  # lead-in flare at the slot mouths
    leg_gap: float = 0.3  # side walls to the legs
    side_wall: float = 0.8  # the fork's side walls (above the board, so they may reach past the outline)
    key_fit: float = 0.1  # around the receiver's tab (1.2 max wide): its roll is then held to about +-3 deg


FP = FrontParams()


def _local(part, sensor):
    (x, y), ang = SENSORS[sensor]
    return Pos(x, y, 0) * Rot(0, 0, ang) * part


def _along_u(r, u0, u1, z, v=0.0):
    return Pos(u0, v, z) * Rot(0, 90, 0) * Cylinder(r, u1 - u0, align=MIN)


def outline(sensor, h=0.3, z0=0.0, p: Params = P):
    """The footprint's casing outline as a prism on the board top, in the robot frame."""
    face = make_face(Polyline(*OUTLINE, close=True))
    return _local(Pos(0, 0, p.board.top_z + z0) * extrude(face, h), sensor)


def _ribs(led: Seat, u0, u1, r, fp: FrontParams):
    """Three crush ribs (top and 45 deg below each side) along [u0, u1] of a bore of radius r."""
    out = None
    depth = fp.fit + fp.rib_interf
    for ang in (0.0, 135.0, -135.0):
        rib = Pos((u0 + u1) / 2, 0, led.z) * Rot(ang, 0, 0) * Pos(0, 0, r - depth / 2 + 0.2) * Box(
            u1 - u0, fp.rib_w, depth + 0.4)
        out = rib if out is None else out + rib
    return out


def _bore_and_ribs(led: Seat, u_front, fp: FrontParams):
    """(bore to cut, ribs to add) for one LED: flange land, body land, hood. Sized on the nominal diameters
    (the ribs take up the tolerance)."""
    rf, rb = led.flange_r + fp.fit, led.body_r + fp.fit
    bore = _along_u(rf, led.flange[0] - 0.01, led.flange[1], led.z) + _along_u(rb, led.flange[1] - 0.01, u_front + 1, led.z)
    ribs = (_ribs(led, led.flange[0], led.flange[1], rf, fp)
            + _ribs(led, led.flange[1] + 0.3, led.dome - 0.3, rb, fp))
    return bore, ribs


def _keyway(led: Seat, key, u_front, fp: FrontParams):
    """Slot for a flange tab, from the flange back to the front (the cap slides over the tab)."""
    dv, dz = key.dir
    w, reach = key.w_max + 2 * fp.key_fit, key.dist_max + fp.key_fit
    return Pos(led.flange[0] - 0.01, 0, led.z) * Rot(degrees(atan2(-dv, dz)), 0, 0) * Box(
        u_front + 1 - led.flange[0], w, reach, align=(Align.MIN, Align.CENTER, Align.MIN))


def sensor_cap(sensor, p: Params = P, fp: FrontParams = FP):
    e, r = EMITTER, RECEIVER
    u_front = max(r.tip, e.tip) + fp.hood
    ro_e, ro_r = e.flange_r + fp.fit + fp.wall, r.flange_r + fp.fit + fp.wall  # outer radii of the sleeves
    # shell: emitter sleeve (reaches back over its flange) and, from the receiver's flange forward, the hull
    # of both sleeves (it closes over the receiver's keyway), on a flat base under the receiver as wide as the
    # outline
    wb = OUTLINE[0][1] - fp.base_margin
    section = make_hull((Pos(0, e.z) * Circle(ro_e)).edges() + (Pos(0, r.z) * Circle(ro_r)).edges()).face()
    shell = (_along_u(ro_e, e.flange[0], u_front, e.z)
             + extrude(Plane.YZ.offset(r.flange[0]) * section, u_front - r.flange[0])
             + Pos(r.flange[0], 0, 0) * Box(u_front - r.flange[0], 2 * wb, r.z, align=(Align.MIN, Align.CENTER, Align.MIN)))
    body = shell
    # fork around the four legs, behind the receiver's flange
    z0, z1 = fp.fork_z
    u_back = e.legs_u - e.leg_w_max / 2 - fp.fork_tail
    fork = Pos(u_back, 0, z0) * Box(r.flange[0] - u_back, 2 * (OUTLINE[2][1] - fp.base_margin), z1 - z0,
                                   align=(Align.MIN, Align.CENTER, Align.MIN))
    for sv in (1, -1):
        fork -= Pos(u_back - 1, sv * LEG_V, z0 - 1) * Box(r.legs_u + r.leg_w_max / 2 + 0.1 - u_back + 1, fp.slot_w, z1 - z0 + 2,
                                                         align=(Align.MIN, Align.CENTER, Align.MIN))
        # lead-in: the slot flares at its open (rear) end
        flare = make_face(Polyline((u_back - 0.01, sv * LEG_V - fp.slot_w / 2 - fp.slot_lead),
                                   (u_back + fp.slot_lead * 1.5, sv * LEG_V - fp.slot_w / 2),
                                   (u_back + fp.slot_lead * 1.5, sv * LEG_V + fp.slot_w / 2),
                                   (u_back - 0.01, sv * LEG_V + fp.slot_w / 2 + fp.slot_lead), close=True))
        fork -= Pos(0, 0, z0 - 1) * extrude(flare, z1 - z0 + 2)
    # side walls along the whole fork (a U-channel around the legs: stiff to print and handle), tying it
    # to the base and the emitter sleeve
    wall_v = LEG_V + LEG_W / 2 + fp.leg_gap
    for sv in (1, -1):
        fork += Pos(u_back, sv * wall_v, z0) * Box(
            r.flange[0] + 0.02 - u_back, fp.side_wall, e.z - 2 - z0,
            align=(Align.MIN, Align.MIN if sv > 0 else Align.MAX, Align.MIN))
    body += fork
    # bores with their ribs; the emitter's pitched about its flange
    eb, er = _bore_and_ribs(e, u_front, fp)
    if fp.emitter_tilt:
        pivot = (sum(e.flange) / 2, 0, e.z)
        tilt = Pos(*pivot) * Rot(0, fp.emitter_tilt, 0) * Pos(-pivot[0], 0, -pivot[2])
        eb, er = tilt * eb, tilt * er
    rb, rr = _bore_and_ribs(r, u_front, fp)
    body -= eb + rb + _keyway(r, RECEIVER_TAB, u_front, fp)
    # open under the receiver (it sits 0.3 mm above the board): the board closes the bore
    body -= Pos(r.flange[0] - 1, 0, -1) * Box(u_front - r.flange[0] + 2, 2 * fp.under_slot, 1 + fp.raise_z,
                                             align=(Align.MIN, Align.CENTER, Align.MIN))
    body += (er + rr) & shell
    body = _local(Pos(0, 0, p.board.top_z) * body, sensor)
    # base: only the outline, where it is over the board, touches the board; the rest is raised
    base_face = make_face(Polyline(*[(u - (fp.base_margin if u > 0 else -fp.base_margin),
                                      v - (fp.base_margin if v > 0 else -fp.base_margin)) for u, v in OUTLINE], close=True))
    base = _local(Pos(0, 0, p.board.top_z - 1) * extrude(base_face, 1 + fp.raise_z), sensor)
    base &= Pos(0, 0, p.board.top_z - 1) * extrude(layout.pcb_face(), 1 + fp.raise_z)
    (x, y), _ = SENSORS[sensor]
    low = Pos(x, y, p.board.top_z - 1) * Box(40, 40, 1 + fp.raise_z, align=MIN)
    body -= low - base
    # drop specks left where the outline runs past the board edge
    solids = [so for so in body.solids() if so.volume > 0.1]
    body = solids[0] if len(solids) == 1 else Compound(solids)
    body.label, body.color = f"sensor_cap_{sensor}", (0.12, 0.12, 0.14)
    return body


@dataclass(frozen=True)
class BumperParams:
    """Bumper: a TPU band hugging the board's front edge and the first part of both diagonal edges. The
    board's nose widens backwards at ~27 deg, so pushing the band on wedges it tight; a lip over the free
    strip in front of the LEDs sets its height. Crash loads go into the board edge."""
    band: float = 3.2  # outwards from the board edge: the bumper leads the diagonal sensor caps
    x_cut: float = 47.5  # the band wraps back along the diagonal edges to here
    bottom_z: float = 0.4  # above the floor (it also closes the skirt across the front)
    top_z: float = 3.15  # below the diagonal caps' raised parts (3.49)
    grip: float = 0.2  # interference on the board edge (press fit; TPU)
    lip_w: float = 0.9  # over the board top, behind the front edge (free strip: parts end at x 52.3)
    lip_t: float = 0.6
    lip_y: float = 10.5  # half length of the lip
    edge_r: float = 0.8  # rounded outer edges (plan view)
    nose_r: float = 1.5  # rounded lower outer edge: it rides up over floor seams instead of catching (the top
    # stays flat: it prints top down)


BP = BumperParams()


def bumper(p: Params = P, bp: BumperParams = BP):
    board = layout.pcb_face()
    front = Pos(bp.x_cut + 50, 0) * Rectangle(100, 200)
    outer = offset(offset(board, bp.band), -bp.edge_r)
    outer = offset(outer, bp.edge_r)
    # the band's lower outer edge is rounded: a core nose_r inside the outer surface, grown by nose_r and cut
    # flat at the top
    r = min(bp.nose_r, bp.top_z - bp.bottom_z - 0.05)
    core = Pos(0, 0, bp.bottom_z + r) * extrude(offset(outer & Pos(bp.x_cut + 45, 0) * Rectangle(100, 200), -r),
                                                bp.top_z - bp.bottom_z)
    body = offset(core, r) & Pos(0, 0, bp.bottom_z) * extrude(front - board, bp.top_z - bp.bottom_z)
    # press fit: the band reaches grip into the board edge, over the board's thickness only
    grip = (board - offset(board, -bp.grip)) & front
    body += Pos(0, 0, p.board.bottom_z) * extrude(grip, p.board.thickness)
    # lip over the board top along the straight front edge
    x_edge = max(v.X for v in board.vertices())
    body += Pos(x_edge - bp.lip_w / 2, 0, p.board.top_z) * Box(bp.lip_w + 0.01, 2 * bp.lip_y, bp.lip_t, align=MIN)
    body.label, body.color = "bumper", (0.15, 0.15, 0.17)
    return body


def parts(p: Params = P, fp: FrontParams = FP, bp: BumperParams = BP):
    out = {}
    for s in SENSORS:
        cap = sensor_cap(s, p, fp)
        out[cap.label] = cap
    out["bumper"] = bumper(p, bp)
    return out
