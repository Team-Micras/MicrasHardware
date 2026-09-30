"""Front: wall-sensor caps (black resin). The nose, front wing and skid are part of the top body (body.py).

Each wall sensor is a stacked pair of 5 mm THT parts with bent legs: an SFH 4550 emitter (half angle 3 deg)
above a TPS601A receiver (10 deg). With beams that narrow, the aim matters far more than any beam shaping
(a 1.5 deg pitch error changes the reading by 10-40 % up close), and an aperture in front of the lenses
only cuts the signal. So the cap:

- stands on the board, on the outline the footprint draws for a sensor casing, and so sets the height,
  pitch and roll of both LEDs from the board;
- holds each LED at two lands (flange and body) with small crush ribs, which absorb the print tolerance;
- locates sideways and in yaw with a fork around the four soldered legs, and axially on the flanges;
- wraps the LED bodies and puts a septum between them (side light and direct crosstalk), with the front
  open at full lens width and a short hood;
- can pitch the emitter down towards the receiver (emitter_tilt) to move the reading's peak closer.

It slides on from the front along the look direction; a drop of glue on the base keeps it down.
"""

from dataclasses import dataclass

from build123d import Align, Box, Compound, Cylinder, Plane, Polyline, Pos, Rot, extrude, make_face

from . import layout
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


@dataclass(frozen=True)
class Led:
    z: float  # optical axis above the board top
    flange: tuple  # (u back, u front)
    dome: float  # u where the lens dome starts
    tip: float  # u of the lens tip
    legs_u: float  # u of the vertical leg run (pads)


# from the KiCad sensor model (WALL_SENSOR_STACKED.STEP)
EMITTER = Led(z=9.75, flange=(-2.32, -1.12), dome=3.93, tip=6.68, legs_u=-5.62)
RECEIVER = Led(z=3.25, flange=(-1.32, -0.12), dome=4.93, tip=7.68, legs_u=-3.12)
FLANGE_R, BODY_R = 2.95, 2.75
LEG_V, LEG_W = 1.27, 0.4
# casing outline drawn by the footprint (silkscreen and User.13), (u, v): the cap may stand on it
OUTLINE = ((7.62, 3.429), (-2.032, 3.429), (-2.032, 2.413), (-6.731, 2.413),
           (-6.731, -2.413), (-2.032, -2.413), (-2.032, -3.429), (7.62, -3.429))
EMITTER_H, DETECTOR_H = EMITTER.z, RECEIVER.z  # kept for older scripts


@dataclass(frozen=True)
class FrontParams:
    fit: float = 0.15  # radial clearance of the bores (the ribs grip)
    rib_w: float = 0.5
    rib_interf: float = 0.05  # crush-rib interference on the LED
    wall: float = 0.6  # around the flange bore
    hood: float = 1.0  # beyond the receiver's lens tip (the front stays open at full lens width)
    emitter_tilt: float = 0.0  # deg, emitter pitched down towards the receiver (bench test 0, 1, 2)
    base_margin: float = 0.08  # base inside the outline
    raise_z: float = 1.45  # the cap outside the outline stays this far above the board (parts up to 1.1)
    under_slot: float = 2.0  # half width of the opening under the receiver (it sits 0.3 above the board)
    fork_z: tuple = (1.35, 2.15)  # tines: clear of the parts behind, below the receiver's leg bend (2.3)
    fork_tail: float = 0.4  # behind the emitter's legs
    slot_w: float = 0.5  # around the 0.4 mm legs


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


def _ribs(led: Led, u0, u1, r, fp: FrontParams):
    """Three crush ribs (top and 45 deg below each side) along [u0, u1] of a bore of radius r."""
    out = None
    depth = fp.fit + fp.rib_interf
    for ang in (0.0, 135.0, -135.0):
        rib = Pos((u0 + u1) / 2, 0, led.z) * Rot(ang, 0, 0) * Pos(0, 0, r - depth / 2 + 0.2) * Box(
            u1 - u0, fp.rib_w, depth + 0.4)
        out = rib if out is None else out + rib
    return out


def _bore_and_ribs(led: Led, u_front, fp: FrontParams):
    """(bore to cut, ribs to add) for one LED: flange land, body land, hood."""
    rf, rb = FLANGE_R + fp.fit, BODY_R + fp.fit
    bore = _along_u(rf, led.flange[0] - 0.01, led.flange[1], led.z) + _along_u(rb, led.flange[1] - 0.01, u_front + 1, led.z)
    ribs = (_ribs(led, led.flange[0], led.flange[1], FLANGE_R + fp.fit, fp)
            + _ribs(led, led.flange[1] + 0.3, led.dome - 0.3, BODY_R + fp.fit, fp))
    return bore, ribs


def sensor_cap(sensor, p: Params = P, fp: FrontParams = FP):
    e, r = EMITTER, RECEIVER
    u_front = r.tip + fp.hood
    ro = FLANGE_R + fp.fit + fp.wall  # outer radius of the sleeves
    top = e.z + ro
    # shell: emitter sleeve (reaches back over its flange) and receiver sleeve, which overlap into one
    # figure-eight, on a flat base under the receiver as wide as the outline
    wb = OUTLINE[0][1] - fp.base_margin
    shell = (_along_u(ro, e.flange[0], u_front, e.z) + _along_u(ro, r.flange[0], u_front, r.z)
             + Pos(r.flange[0], 0, 0) * Box(u_front - r.flange[0], 2 * wb, r.z, align=(Align.MIN, Align.CENTER, Align.MIN)))
    body = shell
    # fork around the four legs, behind the receiver's flange
    z0, z1 = fp.fork_z
    u_back = e.legs_u - LEG_W / 2 - fp.fork_tail
    fork = Pos(u_back, 0, z0) * Box(r.flange[0] - u_back, 2 * (OUTLINE[2][1] - fp.base_margin), z1 - z0,
                                   align=(Align.MIN, Align.CENTER, Align.MIN))
    for sv in (1, -1):
        fork -= Pos(u_back - 1, sv * LEG_V, z0 - 1) * Box(r.legs_u + LEG_W / 2 + 0.1 - u_back + 1, fp.slot_w, z1 - z0 + 2,
                                                         align=(Align.MIN, Align.CENTER, Align.MIN))
    # side plates behind the receiver tie the fork to the base and the emitter sleeve (clear of the legs)
    for sv in (1, -1):
        fork += Pos(e.flange[0], sv * (LEG_V + LEG_W / 2 + 0.33), z0) * Box(
            r.flange[0] + 0.02 - e.flange[0], OUTLINE[2][1] - fp.base_margin - (LEG_V + LEG_W / 2 + 0.33), e.z - 2 - z0,
            align=(Align.MIN, Align.MIN if sv > 0 else Align.MAX, Align.MIN))
    body += fork
    # bores with their ribs; the emitter's pitched about its flange
    eb, er = _bore_and_ribs(e, u_front, fp)
    if fp.emitter_tilt:
        pivot = (sum(e.flange) / 2, 0, e.z)
        tilt = Pos(*pivot) * Rot(0, fp.emitter_tilt, 0) * Pos(-pivot[0], 0, -pivot[2])
        eb, er = tilt * eb, tilt * er
    rb, rr = _bore_and_ribs(r, u_front, fp)
    body -= eb + rb
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


def parts(p: Params = P, fp: FrontParams = FP):
    out = {}
    for s in SENSORS:
        cap = sensor_cap(s, p, fp)
        out[cap.label] = cap
    return out
