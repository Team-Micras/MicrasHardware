"""Front: the wall-sensor caps (resin, painted black).

Each wall sensor is a stacked pair of THT parts with bent legs: an SFH 4550 emitter (5 mm epoxy, half angle
3 deg) above a TPS601A receiver (TO-18 can with a lens and a key tab, 10 deg); leds.py models both from their
datasheets. With beams that narrow, the aim matters far more than any beam shaping
(a 1.5 deg pitch error changes the reading by 10-40 % up close), and an aperture in front of the lenses
only cuts the signal. So the cap:

- stands on the board, on the outline the footprint draws for a sensor casing, and so sets the height,
  pitch and roll of both LEDs from the board;
- holds each LED at two lands (flange and body) with small crush ribs, which absorb the print tolerance;
- locates axially on the flanges; sideways and in yaw it follows the LEDs it grips, with its base lined up on the
  footprint's outline before the glue (a fork round the soldered legs, behind the receiver, blocked the slide-on:
  FrontParams.fork, off);
- keys the receiver's roll with a short keyway for its flange tab, at the back only (the cap slides on from the
  front, so the tab only ever enters the back end; a keyway on to the front would let light in by the lens);
- has a flat in the emitter's flange bore for the flange's flat (loose: it only keeps the wrong roll out);
- wraps the LED bodies and puts a septum between them (side light and direct crosstalk), with the front
  open at full lens width and a short hood;
- can pitch the emitter down towards the receiver (emitter_tilt) to move the reading's peak closer.

It slides on from the front along the look direction; a drop of glue on the base keeps it down.
"""

from dataclasses import dataclass
from math import atan2, degrees

from build123d import (Align, Box, Circle, Compound, Cylinder, Plane, Polyline, Pos, Rot, extrude, make_face,
                       make_hull, offset)

from .leds import EMITTER, EMITTER_FLAT, LEG_V, RECEIVER, RECEIVER_TAB, Seat
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
    fit: float = 0.175  # radial clearance of the bores (the ribs grip): 0.15 at 2.0 s, + 0.025 for 2.5 s
    rib_w: float = 0.5
    rib_interf: float = 0.10  # crush-rib interference on the LED: the test caps' 1-3 dots compared 0.05 / 0.10 / 0.15
    # at 2.5 s, and 2 dots (0.10) gripped best
    rib_low: float = 135.0  # deg from the top: the two lower ribs (the emitter's)
    rib_low_receiver: float = 110.0  # the receiver's stay above the opening under it
    wall: float = 0.8  # around the flange bore (0.6 looked fragile)
    hood: float = 1.0  # beyond the receiver's lens tip (the front stays open at full lens width)
    emitter_tape: float = 0.2  # radial, on the emitter's body (not its flange): one wrap of electrical tape (about
    # 0.18 mm) round the epoxy against side light; the body bore and its ribs grow by this
    emitter_tilt: float = 2.0  # deg, emitter pitched down towards the receiver (bench test of 0, 1, 2: 2 worked well)
    base_margin: float = 0.08  # base inside the outline
    raise_z: float = 1.45  # the cap outside the outline stays this far above the board (parts up to 1.1)
    under_slot: float = 2.0  # half width of the opening under the receiver (its flange is 0.45 above the board, too
    # close for a floor under it): straight-sided, from the board up into the bore
    fork: bool = False  # the leg fork (see the module doc): off, it blocks the slide-on
    fork_z: tuple = (1.35, 2.15)  # tines: clear of the parts behind, below the receiver's leg bend (2.58)
    fork_tail: float = 0.4  # behind the emitter's legs
    slot_w: float = 0.8  # around the legs (0.6 max; bent by hand: +-0.1; resin closes slots a little)
    slot_lead: float = 0.6  # lead-in flare at the slot mouths
    leg_gap: float = 0.3  # side walls to the legs
    side_wall: float = 0.8  # the fork's side walls (above the board, so they may reach past the outline)
    key_cover: float = 0.6  # wall over the keyway
    flat_gap: float = 0.15  # the emitter flange bore's flat (for the flange's flat), clear of it by this much beyond
    # the fit: a loose key (the flat isn't dimensioned on the datasheet)
    key_fit: float = 0.125  # around the receiver's tab (1.2 max wide): its roll is then held to about +-3 deg


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


def _ribs(led: Seat, u0, u1, r, fp: FrontParams, low):
    """Three crush ribs (on top and `low` deg from the top each side) along [u0, u1] of a bore of radius r."""
    out = None
    depth = fp.fit + fp.rib_interf
    for ang in (0.0, low, -low):
        rib = Pos((u0 + u1) / 2, 0, led.z) * Rot(ang, 0, 0) * Pos(0, 0, r - depth / 2 + 0.2) * Box(
            u1 - u0, fp.rib_w, depth + 0.4)
        out = rib if out is None else out + rib
    return out


def _bore_and_ribs(led: Seat, u_front, fp: FrontParams, low, wrap=0.0):
    """(bore to cut, ribs to add) for one LED: flange land, body land, hood. Sized on the nominal diameters
    (the ribs take up the tolerance); `wrap`: anything round the body (tape), radial."""
    rf, rb = led.flange_r + fp.fit, led.body_r + fp.fit + wrap
    # (the flange land runs out the back: pitched, it would leave a sliver at the back face)
    bore = _along_u(rf, led.flange[0] - 0.5, led.flange[1], led.z) + _along_u(rb, led.flange[1] - 0.01, u_front + 1, led.z)
    ribs = (_ribs(led, led.flange[0], led.flange[1], rf, fp, low)
            + _ribs(led, led.flange[1] + 0.3, led.dome - 0.3, rb, fp, low))
    return bore, ribs


def _keyway(led: Seat, key, fp: FrontParams, grow=0.0):
    """Slot for a flange tab, open at the cap's back face, just past the flange's front face (the tab is only on
    the flange). grow > 0 gives the cover over it (a wall all round the slot, its front end included)."""
    dv, dz = key.dir
    w, reach = key.w_max + 2 * fp.key_fit + 2 * grow, key.dist_max + fp.key_fit + grow
    u1 = led.flange[1] + 2 * fp.key_fit + grow
    return Pos(led.flange[0] - 0.01, 0, led.z) * Rot(degrees(atan2(-dv, dz)), 0, 0) * Box(
        u1 - led.flange[0], w, reach, align=(Align.MIN, Align.CENTER, Align.MIN))


def sensor_cap(sensor, p: Params = P, fp: FrontParams = FP):
    e, r = EMITTER, RECEIVER
    u_front = max(r.tip, e.tip) + fp.hood
    ro_e, ro_r = e.flange_r + fp.fit + fp.wall, r.flange_r + fp.fit + fp.wall  # outer radii of the sleeves
    # shell: emitter sleeve (reaches back over its flange) and, from the receiver's flange forward, the hull
    # of both sleeves, on a flat base under the receiver as wide as the
    # outline
    wb = OUTLINE[0][1] - fp.base_margin
    section = make_hull((Pos(0, e.z) * Circle(ro_e)).edges() + (Pos(0, r.z) * Circle(ro_r)).edges()).face()
    shell = (_along_u(ro_e, e.flange[0], u_front, e.z)
             + extrude(Plane.YZ.offset(r.flange[0]) * section, u_front - r.flange[0])
             + Pos(r.flange[0], 0, 0) * Box(u_front - r.flange[0], 2 * wb, r.z, align=(Align.MIN, Align.CENTER, Align.MIN)))
    # a wall over the receiver's keyway (the hull alone leaves a 0.1 mm skin there: it would tear or let IR in)
    shell += _keyway(r, RECEIVER_TAB, fp, grow=fp.key_cover)
    body = shell
    # fork around the four legs, behind the receiver's flange: off. It sat inside the receiver's outline seen from
    # the front, so the cap could not slide on over the receiver (nor take the LEDs from behind); nothing of the cap
    # may lie behind either flange inside its outline (tools/check_sensors.py sweeps them)
    if fp.fork:
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
    eb, er = _bore_and_ribs(e, u_front, fp, fp.rib_low, fp.emitter_tape)
    # a flat in the flange bore, opposite the flange's flat (cathode side, -v), with room to spare
    dv, _ = EMITTER_FLAT.dir
    d_flat = EMITTER_FLAT.dist + fp.fit + fp.flat_gap
    ef = Pos(e.flange[0] - 0.01, dv * (d_flat + 2), e.z) * Box(e.flange[1] - e.flange[0] + 0.01, 4, 2 * e.flange_r,
                                                              align=(Align.MIN, Align.CENTER, Align.CENTER))
    if fp.emitter_tilt:
        pivot = (sum(e.flange) / 2, 0, e.z)
        tilt = Pos(*pivot) * Rot(0, fp.emitter_tilt, 0) * Pos(-pivot[0], 0, -pivot[2])
        eb, er, ef = tilt * eb, tilt * er, tilt * ef
    rb, rr = _bore_and_ribs(r, u_front, fp, fp.rib_low_receiver)
    body -= eb + rb + _keyway(r, RECEIVER_TAB, fp)
    # the receiver's flange passes under the emitter sleeve as the cap slides on: its bore runs on out the back
    body -= _along_u(r.flange_r + fp.fit, r.flange[0] - 10, r.flange[0] + 0.01, r.z)
    # open under the receiver, straight up into its bore (the bore would leave a 0.3 mm skin over the board): the
    # board closes the bore
    under = Pos(r.flange[0] - 1, 0, -1) * Box(u_front - r.flange[0] + 2, 2 * fp.under_slot, 1 + r.z,
                                             align=(Align.MIN, Align.CENTER, Align.MIN))
    body -= under
    body += (er + rr + ef) & (shell - under)
    body = _local(Pos(0, 0, p.board.top_z) * body, sensor)
    # base: only the outline touches the board (both legs, also where the board's edge runs under them); the rest
    # is raised
    base_face = make_face(Polyline(*[(u - (fp.base_margin if u > 0 else -fp.base_margin),
                                      v - (fp.base_margin if v > 0 else -fp.base_margin)) for u, v in OUTLINE], close=True))
    base = _local(Pos(0, 0, p.board.top_z - 1) * extrude(base_face, 1 + fp.raise_z), sensor)
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
