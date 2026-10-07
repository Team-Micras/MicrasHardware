"""Printed drivetrain parts: bearing block base + cap per side, magnet cups, wheels.

Each side's block is split at the axle plane (z = axle_z). The base is screwed to the board through
the two countersunk holes; the cap clamps the inner bearing (and, on the left, the level motor)
with two screws into inserts in the base. The right cap also carries the clamp ring of the raised motor. A tube
on the base (one piece with it, not split) reaches out from the block through the wheel's gear into its hollow drum
and holds the outer bearing at its end, pressed in from the end up to a step (Params.AxialStack).

Layout.blocks = "solid" (the default) makes each block one piece instead: the inner bearing presses in from the inboard end
(through the magnet cup's room) up to the lip, and both motor rings are slit clamps (the left one on its rear side).
The motors sit straight in their rings (the eccentric sleeves are gone: the thin rings let the battery sit lower).
"""

from dataclasses import dataclass
from math import atan2, cos, degrees, radians, sin, tan

from build123d import (Align, Box, Circle, Cone, Cylinder, Line, Plane, Pos, Rectangle, RegularPolygon, Rot,
                       ThreePointArc, Wire, extrude, make_face, make_hull, mirror, offset)

from .layout import board_boxes
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class DriveParams:
    wall: float = 1.2  # around the bearing bores
    motor_wall: float = 1.5  # the motor rings (2.1-2.7 when they were drawn for the sleeves)
    bearing_fit: float = 0.20  # diametral clearance on the bearing OD in the split pocket: it drops into the base;
    # the split closes by split_relief, which leaves a light clamp (at 2.5 s: the cap closes, no rattle; 0.13 stopped
    # the cap)
    # fits for ABS-Like Pro 2 at 2.5 s (docs/printing.md): outsides print true; the holes printed about as at 2.0 s
    # (the motor and the magnet came out loose with 0.05 added), except the bearing pockets in the blocks
    pillar_wall: float = 1.0  # around the glued cap-screw inserts
    plate_t: float = 2.4  # outer plate skin where it is pocketed from the inboard side
    pocket_rim: float = 1.0  # solid rim around the pockets, along the plate's outline
    pocket_margin: float = 0.8  # pockets stay this far from the rings, screw columns and housing web
    split_relief: float = 0.05  # taken off the cap's split face: tightening the cap clamps bearings and motor
    # (0.15 with the holes printing small crushed the bearings: the split blocks fitted worst)
    # the motor's fits (diametral; the clamp or the cap holds it), from the parts printed at 2.5 s:
    ring_motor_fit_split: float = 0.06  # no sleeves, split blocks: the level (left) motor, clamped by the cap (0.10
    # was a little loose)
    ring_motor_fit_split_upper: float = 0.15  # no sleeves, split blocks: the raised (right) motor in its slit ring
    # (0.10, printed in round 4, should be bigger; the slit clamp closes the rest)
    ring_motor_fit_solid: float = 0.15  # no sleeves, one-piece blocks (0.10 needed too much force: the ring has no
    # cap to open)
    seat_front: float = 0.6  # the ring runs on this far past the motor's front face
    stop_ledge: float = 1.0  # radial: the flat ring the motor's front face sits on, then the 50 deg cone (a wider flat
    # end would be a ceiling inside the bore as it prints, and sag)
    seat_min_y: float = 1.0  # the rings run inboard past the encoder daughterboards (slotted round them) to just short
    # of the centre line (the two rings would meet past it); 7.6, outboard of the encoder boards, gripped 7.4 mm
    clamp_min_y: float = 7.6  # the one-piece left clamp's ear starts here at most (further in if its nut needs it)
    pad_inset: float = 0.3  # stay inside the silkscreen contact outline
    lift: float = 1.8  # everything except the pads starts this far above the board top
    pad_h: float = 3.0  # contact plate thickness
    insert_d: float = 3.35  # threaded insert hole in resin (M2x2 OD 3.2, glued in): goes in without forcing
    insert_l: float = 2.0
    cap_insert_depth: float = 2.3  # the cap screws' insert pockets in the split bases: deeper than the insert, so it
    # seats fully below the split face with room for the glue (2.0 was too shallow)
    screw_clear_d: float = 2.2
    screw_l: float = 5.0
    csk_d: float = 4.0  # countersink for the M2 flat heads in the caps
    cap_screw_y: float = 16.0  # |y| of the cap screws, at most (the head stays 0.3 clear of the gear cut-out)
    cap_screw_front_x: float = 5.2  # clear of the fan mount feet; the rear screw's x is found per side (cap_screws)
    frame_screw_depth: float = 3.8  # M2x5 through a 1.2 frame floor
    frame_boss_top_left: float = 22.0  # keeps the screw tip 0.5 above the left seat bore
    tray_bottom_z: float = 28.0  # frame tray underside, sits on the right boss (0.22 over the raised motor's ring;
    # 29.0 with the rings drawn for the sleeves)
    slit: float = 0.8  # clamp slit in the motor rings (the raised one; both in the one-piece blocks)
    ear: float = 2.2  # clamp ear thickness either side of the slit (M2x5 then engages 2 mm)
    clamp_nut_wall: float = 0.6  # resin between a ring clamp's nut trap and the motor bore: the screw as close to the
    # motor as that allows (its nut was all outside the ring: the ears reached 1.1 mm further out)
    # ear on each cap for the fan mount's arm: in front of the front cap screw (clear of its head), top level
    # with the left cap's front column; an M2 screw comes down through the arm's tab into a nut trapped under it
    fan_ear: tuple = (10.2, 14.2)  # (x, |y|) of the screw
    fan_ear_r: float = 3.3
    fan_ear_t: float = 2.9
    fan_tab_r: float = 2.5  # the arm's tab drops into a recess in the ear's top: it locates the fan mount
    fan_recess: float = 0.5
    nut_af: float = 4.15  # M2 nut across flats, with fit (4.10 at 2.0 s; 4.05 needed too much force; glue if loose)
    nut_t: float = 1.6
    # the board screws' nuts (from under the board, an M2x5 reaches about 4 mm above the board top), like the v1
    # blocks: 0.9 mm of resin under the nut (the owner's choice). Below the lift the pad must stay inside the
    # silkscreen tab (2.7 mm radius round the hole), so the nut's lower 0.9 mm has thin walls at its corners (~0.3);
    # above the lift a boss holds it. It slides in from the side: a drop-in well from above, as in v1, would cut the
    # bearing housing over the front holes
    board_nut_z: float = 0.9  # nut bottom above the board top
    board_nut_wall: float = 1.0  # round the nut's corners, in the boss
    board_nut_roof: float = 0.6
    # the battery basket's feet (frame.py): (side, x) on each cap's outer plate, on its mid-plane; a peg on the
    # foot drops into a socket in the cap (it places the basket before its screws go in), "pad" feet only rest
    # (the rear peg sits on the left frame boss's flat top: at -15 its socket ran into the rear cap screw's countersink)
    basket_feet: tuple = ((1, 0.0, "peg"), (1, -10.75, "peg"), (-1, -1.0, "pad"))
    socket_d: float = 2.4  # (2.2: the PLA pegs were tight)
    socket_depth: float = 2.0
    socket_land_r: float = 2.2  # a flat landing each side of each socket, across the plate, at the lowest point of the
    # top there (the foot, Ø4, stands on it; on a slope the socket opened lopsided)
    solid_bearing_fit: float = 0.05  # full rings (the tube, the one-piece block): the bearing presses in with a
    # firm thumb push (the fit bar fit_BRG at 2.5 s: it slid into 5.10 with no force)


D = DriveParams()


# ---- helpers --------------------------------------------------------------------------------

def countersunk(d: "DriveParams" = None, depth=20.0, up=20.0):
    """Cutter for an M2 flat-head screw: head seat at the origin, head side towards +z.

    The screw's top face sits at z=0; its shank runs `depth` towards -z. A counterbore of the head
    diameter clears everything above the seat (`up`).
    """
    d = d or D
    h = (d.csk_d - d.screw_clear_d) / 2
    cutter = Pos(0, 0, -h) * Cone(d.screw_clear_d / 2, d.csk_d / 2, h, align=MIN)
    cutter += Cylinder(d.csk_d / 2 + 0.05, up, align=MIN)
    cutter += Cylinder(d.screw_clear_d / 2, depth, align=MAX)
    return cutter


def along_y(radius, y0, y1, x=0.0, z=0.0):
    """Cylinder with its axis parallel to y, spanning y0..y1."""
    lo, hi = min(y0, y1), max(y0, y1)
    return Pos(x, lo, z) * Rot(-90, 0, 0) * Cylinder(radius, hi - lo, align=MIN)


def race_cone(inside, gap, p: Params = P):
    """The cone a rotating part bears on an inner race with, along +z: boss_d at z = 0 (the race), cone_d at
    z = inside (0.1 past the housing's face, so the flare stays clear of its bore's edge), then 45 deg out to the
    part's face at z = inside + gap."""
    st = p.stack
    r0, r1 = st.boss_d / 2, st.cone_d / 2
    cone = Cone(r0, r1, inside, align=MIN)
    return cone + Pos(0, 0, inside) * Cone(r1, r1 + gap, gap, align=MIN)


def contact_zone(p: Params = P, inset=D.pad_inset):
    """Left-side silkscreen contact zone (x, y) as a face on the board top, inset by `inset`."""
    b = p.board
    fx, fy = b.front_hole
    rx, ry = b.rear_hole
    ny = b.notch_inner_y
    r = 3.0 - inset
    pts_front = [(fx + r, fy), (fx + r, ny - inset)]
    wire = Wire([
        Line((fx + r, fy), (fx + r, ny - inset)),
        Line((fx + r, ny - inset), (rx + 3.0 - inset, ny - inset)),
        Line((rx + 3.0 - inset, ny - inset), (rx + r, ry)),
        ThreePointArc((rx + r, ry), (rx, ry + r), (rx - r, ry)),
        Line((rx - r, ry), (rx - r, 13.75 + inset)),
        Line((rx - r, 13.75 + inset), (fx - r, 13.75 + inset)),
        Line((fx - r, 13.75 + inset), (fx - r, fy)),
        ThreePointArc((fx - r, fy), (fx, fy - r), (fx + r, fy)),
    ])
    return make_face(wire)


def solid(p=P):
    return p.layout.blocks == "solid"


def seat_d(p=P, d=D, angle=None):
    """The motor seat's bore; `angle` picks the raised (right) motor's own fit in the split blocks."""
    if solid(p):
        return p.motor.d + d.ring_motor_fit_solid
    upper = angle is not None and angle == p.layout.motor_angle_right
    return p.motor.d + (d.ring_motor_fit_split_upper if upper else d.ring_motor_fit_split)


def ring_r(angle, p=P, d=D):
    """Outer radius of a motor ring."""
    return seat_d(p, d, angle) / 2 + d.motor_wall


def seat_span(p=P, d=D):
    """|y| extent of a motor ring (inner, outer)."""
    return d.seat_min_y, p.motor_front_y + d.seat_front


def seat_stop(p=P, d=D):
    """|y| of the motor's front face, where it stops in the seat."""
    return p.motor_front_y


def seat_cut(angle, p=P, d=D):
    """Cutter for a motor seat: the bore, open inboard, ending in a flat ledge (stop_ledge wide) that the motor's
    front face sits on, then a cone (50 deg, so it prints without supports inside the bore) that narrows to the
    pinion's clearance."""
    mx, mz = motor_axis(angle, p)
    sy0, _ = seat_span(p, d)
    stop = seat_stop(p, d)
    r0 = seat_d(p, d, angle) / 2
    r_ledge = r0 - d.stop_ledge
    r1 = p.gears.tip_d(p.gears.pinion_z) / 2 + 0.4  # the pinion's clearance
    length = (r_ledge - r1) * tan(radians(50))
    cone = Pos(mx, stop - 0.01, mz) * Rot(-90, 0, 0) * Cone(r_ledge, r1, length + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return along_y(r0, sy0 - 1, stop, mx, mz) + cone


def encoder_slot(angle, p=P, d=D):
    """Cutter for the slot a (left-frame) motor ring straddles its encoder daughterboard with, open below: the
    board (with Layout.clearance round it), only within reach of the ring."""
    from .layout import encoders
    c = p.layout.clearance
    bb = encoders(p)["encoder_pcb_L"].bounding_box()
    slot = Pos(bb.min.X - c, bb.min.Y - c, p.board.top_z - 1) * Box(
        bb.size.X + 2 * c, bb.size.Y + 2 * c, bb.max.Z + c - p.board.top_z + 1, align=(Align.MIN, Align.MIN, Align.MIN))
    mx, mz = motor_axis(angle, p)
    return slot & along_y(ring_r(angle, p, d) + 0.5, bb.min.Y - 1, bb.max.Y + 1, mx, mz)


def motor_axis(angle, p=P):
    """Motor axis (x, z) for a motor at `angle` around the wheel axle."""
    a = radians(angle)
    cd = p.gears.center_distance
    return cd * cos(a), p.axle_z + cd * sin(a)


def housing_span(p=P):
    """|y| extent of the bearing housing in the block (its inboard face to the lip; the tube carries on)."""
    return p.shoulder_y, p.housing_end_y


def tube(p=P, d=None):
    """The outer bearing's tube, from the block's outer face to its end (axis on the axle, robot frame, left)."""
    d = d or D
    st, az = p.stack, p.axle_z
    y0, y1 = p.housing_end_y, p.tube_end_y
    t = along_y(st.tube_od / 2, y0 - 0.01, y1, 0, az)
    t -= along_y(st.shoulder_id / 2, y0 - 1, y1 + 1, 0, az)
    t -= along_y((p.bearing.od + d.solid_bearing_fit) / 2, p.bearing_outer_y - p.bearing.w, y1 + 1, 0, az)
    return t


# ---- parts ----------------------------------------------------------------------------------

def _hull_at(circles, y):
    """Convex hull of circles (x, z, r) as a face in the plane at this y."""
    edges = []
    for x, z, r in circles:
        edges += (Pos(x, z) * Circle(r)).edges()
    face = make_hull(edges) if len(circles) > 1 else Pos(circles[0][0], circles[0][1]) * Circle(circles[0][2])
    return Pos(0, y, 0) * (Plane.XZ * face.face())


def _hull_2d(circles):
    """Convex hull of circles (x, z, r) as a face in the (x, z) plane."""
    edges = []
    for x, z, r in circles:
        edges += (Pos(x, z) * Circle(r)).edges()
    return make_hull(edges).face()


def _belt(circles, y0, y1):
    """Convex hull of circles (x, z, r) in the xz plane, extruded over y0..y1."""
    return extrude(_hull_at(circles, y1), y1 - y0)


def _left_block_solid(p: Params, d: DriveParams, motor_angle):
    """Unsplit left-side block (y > 0). Right side is built by mirroring with its own motor angle."""
    b = p.board
    az = p.axle_z
    rh = p.bearing.od / 2 + d.wall
    hy0, hy1 = housing_span(p)
    sy0, sy1 = seat_span(p, d)
    rs = ring_r(motor_angle, p, d)
    mx, mz = motor_axis(motor_angle, p)

    # contact plate on the silkscreen zone (the board screws go into it from below)
    body = Pos(0, 0, b.top_z) * extrude(contact_zone(p, d.pad_inset), d.pad_h)
    # bearing housing and motor seat as one body: the hull of both rings over the length where they
    # overlap; beyond it each continues as its own ring, with flat, solid end faces
    ring_h, ring_s = (0, az, rh), (mx, mz, rs)
    # (fuse: `+` dropped the seat ring's inboard part for some ring sizes; the hull is drawn 0.05 inside the seat
    # ring, as its faces tangent to the ring's dropped the ring's inboard part too)
    for piece in (along_y(rh, hy0, hy1, 0, az), along_y(rs, sy0, sy1, mx, mz),
                  _belt((ring_h, (mx, mz, rs - 0.05)), max(hy0, sy0), min(hy1, sy1))):
        body = body.fuse(piece).clean()
    # web from the housing down towards the pads (over the housing only: nothing next to the encoders)
    lz = b.top_z + d.lift
    body += Pos(0, (hy0 + hy1) / 2, lz) * Box(2 * rh, hy1 - hy0, az - lz, align=MIN)
    # outer plate: one slab across the block (like the v1 bearing blocks), from the pads up to the rings,
    # carrying the two cap screws; its section is the hull of both rings and the screw bosses
    side = 1 if motor_angle == p.layout.motor_angle_left else -1
    rp = d.insert_d / 2 + d.pillar_wall
    screws = cap_screws(side, p, d)
    y0, y1 = screws[0][1] - rp, hy1
    section = [ring_h, ring_s]
    for sx, sy in screws:
        # start above any board part under or next to the boss (the plate then hangs from the rings)
        z0 = lz
        for bb in board_boxes():
            (bx0, by0, _), (bx1, by1, bz1) = bb["min"], bb["max"]
            by0, by1 = sorted((side * by0, side * by1))
            gx = max(bx0 - sx, 0, sx - bx1)
            gy = max(by0 - sy, 0, sy - by1)
            if (gx * gx + gy * gy) ** 0.5 < rp + 0.3 and not bb["label"].startswith("encoder"):
                z0 = max(z0, bz1 + 0.3)
        # the boss reaches up into the seat ring when the ring is raised above it and they overlap by more than the
        # ring's wall (a near-tangent join left a sliver, which the clamp's slit then cut off as a flake)
        top = az + rh
        near = abs(sx - mx) - rp
        if mz > top and near < rs - d.motor_wall:
            top = max(top, mz - (rs ** 2 - near ** 2) ** 0.5 + 0.8)
        # square bottom corners (they sit flat above the board), round top
        section += [(sx - rp + 0.01, z0 + 0.01, 0.01), (sx + rp - 0.01, z0 + 0.01, 0.01), (sx, top - rp, rp)]
    # the frame boss's top joins the plate's outline when no cap screw is next to it (else it would bury
    # that screw's head in a deep well); block() adds the boss itself
    fx, _, ftop = frame_boss(side, p, d)
    rf = d.insert_d / 2 + d.wall
    if all(abs(sx - fx) >= rf + d.csk_d / 2 + 0.3 for sx, _ in screws):
        section += [(fx - rf + 0.01, ftop - 0.01, 0.01), (fx + rf - 0.01, ftop - 0.01, 0.01)]
    body += _belt(section, y0, y1)
    # lightening pockets in the plate's inboard (hidden) face, leaving the outer skin, a rim along the
    # outline, and full depth at the rings, the screw columns and the web under the housing; they open on
    # the base's top (the split) or have the rings above them, so they print without supports
    pocket = offset(_hull_2d(section), -d.pocket_rim)
    for keep in [Pos(0, az) * Circle(rh), Pos(mx, mz) * Circle(rs), Pos(0, (lz + az) / 2) * Rectangle(2 * rh, az - lz)] + [
            Pos(sx, 50) * Rectangle(2 * rp, 200) for sx, _ in screws] + [
            Pos(fx, 50) * Rectangle(2 * d.socket_land_r, 200) for fs, fx, kind in d.basket_feet  # under the sockets
            if fs == side and kind == "peg"]:
        pocket -= offset(keep, d.pocket_margin)
    for f in pocket.faces():
        if f.area > 2.0:
            body -= extrude(Pos(0, y1 - d.plate_t, 0) * (Plane.XZ * f), y1 - d.plate_t - y0 + 1)
    # round ends in plan around the two screws (the front one clears the fan mount's legs)
    for sx, sy in screws:
        out = 1 if sx > 0 else -1
        end = Pos(sx, sy, lz) * Box(2 * rp, 2 * rp, 40, align=(Align.MIN if out > 0 else Align.MAX, Align.CENTER, Align.MIN))
        body -= end - Pos(sx, sy, lz) * Cylinder(rp, 40, align=MIN)
    # clear the board parts under the plate
    for bb in board_boxes():
        (bx0, by0, _), (bx1, by1, bz1) = bb["min"], bb["max"]
        by0, by1 = sorted((side * by0, side * by1))
        if bz1 + 0.3 > lz and by1 + 0.3 > y0 and by0 - 0.3 < y1 and not bb["label"].startswith("encoder"):
            body -= Pos(bx0 - 0.3, by0 - 0.3, b.top_z - 1) * Box(
                bx1 - bx0 + 0.6, by1 - by0 + 0.6, bz1 + 1.3 - b.top_z, align=(Align.MIN, Align.MIN, Align.MIN))
    return body, motor_angle


def cup_room(p: Params = P):
    """Cutter for the magnet cup's room (it turns in it; drawn for the bigger magnet, so either cup fits): its head
    inboard of the housing, its sleeve inside the housing up to the shoulder."""
    st, az = p.stack, p.axle_z
    hy0 = housing_span(p)[0]
    y_shoulder = p.bearing_inner_y - st.bearing_play - st.shoulder
    return (along_y(max(p.magnet.d, W.alt_magnet[0]) / 2 + st.cup_wall + st.holder_gap, 0, hy0, 0, az)
            + along_y(st.sleeve_d / 2 + st.holder_gap, hy0 - 1, y_shoulder, 0, az))


def board_holes(p: Params = P, d: DriveParams = D):
    """[(x, |y|, angle)] of the board screws (left frame), with the direction their nut's slot opens (deg):
    outward, from the contact zone's middle through the hole."""
    cz = contact_zone(p, d.pad_inset).center()
    return [(hx, hy, degrees(atan2(hy - cz.Y, hx - cz.X))) for hx, hy in (p.board.front_hole, p.board.rear_hole)]


def _cut_left_bores(body, p: Params, d: DriveParams, motor_angle):
    b = p.board
    az = p.axle_z
    hy0, hy1 = housing_span(p)
    sy0, sy1 = seat_span(p, d)
    mx, mz = motor_axis(motor_angle, p)
    st = p.stack
    # the inner bearing's bore, behind a lip at the block's outer face (its bore clears the inner race): the split
    # blocks also have a shoulder inboard; the one-piece blocks take the bearing in from the inboard end, through
    # the room for the magnet cup's sleeve (wider than the bearing), up to the lip
    body -= along_y(st.shoulder_id / 2, hy0 - 1, hy1 + 1, 0, az)
    play = st.bearing_play
    if solid(p):
        rb = (p.bearing.od + d.solid_bearing_fit) / 2
        body -= along_y(rb, hy0 - 1, p.bearing_inner_y + p.bearing.w, 0, az)
    else:
        rb = (p.bearing.od + d.bearing_fit) / 2
        body -= along_y(rb, p.bearing_inner_y - play, p.bearing_inner_y + p.bearing.w + play, 0, az)
    body -= cup_room(p)
    # motor seat bore, open inboard; the motor's front face stops on its ledge
    stop = seat_stop(p, d)
    body -= seat_cut(motor_angle, p, d)
    body -= encoder_slot(motor_angle, p, d)
    # clearance for the pinion and the motor can beyond the seat
    body -= along_y(p.gears.tip_d(p.gears.pinion_z) / 2 + 0.4, stop - 0.01, 30, mx, mz)
    body -= along_y(p.motor.d / 2 + 0.4, -30, sy0, mx, mz)
    # wheel gear and tire clearance
    body -= along_y(p.gears.tip_d(p.gears.wheel_z) / 2 + 0.5, p.gear_y - p.stack.lip_gap, 40, 0, az)
    # board screws: from under the board into an M2 nut trapped board_nut_z up (a boss round it from the lift up,
    # where the base may be wider than the contact zone), slid in sideways through a slot that opens away from the block
    z_nut = b.top_z + d.board_nut_z
    for hx, hy, a in board_holes(p, d):
        r_boss = d.nut_af / 3 ** 0.5 + d.board_nut_wall
        boss = Pos(hx, hy, b.top_z + d.lift) * Cylinder(r_boss, d.board_nut_z + d.nut_t + d.board_nut_roof - d.lift, align=MIN)
        body = body.fuse(boss).clean()
        body -= Pos(hx, hy, b.top_z - 0.01) * Cylinder(d.screw_clear_d / 2, d.screw_l - b.thickness + 0.8, align=MIN)
        body -= Pos(hx, hy, z_nut) * Rot(0, 0, a) * nut_trap(d, d.nut_t + 0.1)  # flats along the slot
        body -= Pos(hx, hy, z_nut) * Rot(0, 0, a) * Box(15, d.nut_af, d.nut_t + 0.1, align=(Align.MIN, Align.CENTER, Align.MIN))
    return body


def cap_screws(side, p: Params = P, d: DriveParams = D):
    """(x, |y|) of the two cap screws on one side. The front one sits in front of the housing; the rear one
    is moved back until its insert clears the motor seat bore and its screwdriver clears the pinion. (The
    one-piece blocks have no cap screws, but keep their columns in the plate.)"""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    sx, sz = mx, mz = motor_axis(angle, p)
    rs = seat_d(p, d, angle) / 2 + 0.3
    r_ins = d.insert_d / 2
    r_pin = p.gears.tip_d(p.gears.pinion_z) / 2 + 0.3
    az = p.axle_z
    x = -d.cap_screw_front_x
    for _ in range(300):
        # insert pocket (az - cap_insert_depth .. az) vs the seat bore circle, in the xz plane
        dz = max(0.0, abs(sz - (az - d.cap_insert_depth / 2)) - d.cap_insert_depth / 2)
        clear_seat = ((x - sx) ** 2 + dz ** 2) ** 0.5 >= rs + r_ins
        # a Ø2.6 driver straight down onto the head vs the pinion above it
        clear_pinion = mz < az or abs(x - mx) >= r_pin + 1.3
        # ... and its head's clearance (cut up from the head) 0.3 off the seat ring, where the ring reaches over the
        # screw's y (1.5 from the axis let it clip a lens off the ring)
        z_head = az + d.screw_l - d.insert_l
        clear_ring = ((x - sx) ** 2 + (max(z_head, sz) - sz) ** 2) ** 0.5 >= ring_r(angle, p, d) + d.csk_d / 2 + 0.35
        if clear_seat and clear_pinion and clear_ring:
            break
        x -= 0.1
    y = min(d.cap_screw_y, p.gear_y - p.stack.lip_gap - d.csk_d / 2 - 0.3)  # head clear of the gear cut-out
    return ((d.cap_screw_front_x, y), (round(x, 2), y))


def frame_boss(side, p: Params = P, d: DriveParams = D):
    """(x, |y|, top z) of the frame mounting boss on each cap."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    sx, sz = motor_axis(angle, p)
    if side > 0:  # on top of the level motor's seat
        return sx, 11.0, d.frame_boss_top_left
    # behind the raised motor's ring, far enough out that the hole misses the bore, and inboard enough to stay 0.6 clear
    # of the rear cap screw's head clearance (at 11.0 it shaved a lens off the boss's side)
    fx = sx - (seat_d(p, d, angle) / 2 + d.insert_d / 2 + 0.6)
    rx, ry = cap_screws(side, p, d)[1]
    fy = min(11.0, ry - d.csk_d / 2 - 0.05 - 0.6 - (d.insert_d / 2 + d.wall))
    return fx, fy, d.tray_bottom_z


def ring_clamps(side, p: Params = P, d: DriveParams = D):
    """The slit ring clamps on one side's block: [(angle, out, y_max)] for _ring_clamp."""
    if side < 0:  # the raised motor's ring is all above the split
        return [(p.layout.motor_angle_right, 1, None)]
    if solid(p):  # the level motor's ring has no cap to clamp it: a slit on its rear side, inboard of the outer plate
        return [(p.layout.motor_angle_left, -1, cap_screws(side, p, d)[0][1] - d.insert_d / 2 - d.pillar_wall)]
    return []


def ring_clamp_screw(angle, out, y_max, p: Params = P, d: DriveParams = D):
    """(x, |y|, z) of a ring clamp's screw axis at the slit's mid-plane, and its ear's (y0, y1, height), in the
    left frame."""
    bx, bz = motor_axis(angle, p)
    sy0, sy1 = seat_span(p, d)
    corner = d.nut_af / 3 ** 0.5  # the nut's corners point along y
    if y_max is None:
        # the nut goes in from below, before the cap is fitted: it stays inboard of the bearing housing, whose blend
        # into the ring lies under the ear further out; the ear runs a wall past it each way, and stops short of the
        # front cap screw, so a screwdriver reaches that screw past it
        ymid = housing_span(p)[0] - 0.3 - corner
        ey0, ey1 = ymid - corner - d.wall, min(ymid + corner + d.wall, sy1, cap_screws(-1, p, d)[0][1] - 1.5)
    else:
        # almost to the slit's end (ending level with the slit breaks the cut), the nut a wall in from that end and
        # the ear a wall past it inboard too
        ey1 = y_max - 0.5
        ymid = ey1 - corner - d.wall
        ey0 = min(max(sy0, d.clamp_min_y), ymid - corner - d.wall)
    ear_h = 2 * d.ear + d.slit
    # the nut's flat faces the motor; its top (the nearest the bore comes, from below) is clamp_nut_wall off the bore
    dz = ear_h / 2 - d.nut_t
    rb = seat_d(p, d, angle) / 2
    ex = bx + out * ((rb * rb - dz * dz) ** 0.5 + d.clamp_nut_wall + d.nut_af / 2)
    return (ex, ymid, bz), (ey0, ey1, ear_h)


def _ring_clamp(top, p, d, angle, out=1, y_max=None):
    """Slit a motor ring on its `out` side (+1 front, -1 rear) at the bore's centre height and add a vertical
    clamp screw across the slit, just outside the ring, into an M2 nut trapped under the lower ear. The slit runs
    the ring's length, the ear only outboard of the encoder board (clamp_min_y). `y_max`: the slit and its ear stop
    short of it (the outer plate there would make the clamp too stiff to close)."""
    bx, bz = motor_axis(angle, p)
    sy0, sy1 = seat_span(p, d)
    rb = seat_d(p, d, angle) / 2
    (ex, ymid, _), (ey0, ey1, ear_h) = ring_clamp_screw(angle, out, y_max, p, d)
    if y_max is None:
        hy0, hy1 = housing_span(p)
        y_lo, y_hi = min(sy0, hy0) - 1, max(sy1, hy1) + 1  # through the whole ring and its tapers
    else:
        y_lo, y_hi = sy0 - 1, y_max - 0.3
    # the ear starts inside the bore (re-cut below): a face tangent to the bore breaks the union
    xa, xb = bx + out * (rb - 0.3), ex + out * (d.nut_af / 2 + d.wall)
    top = top.fuse(Pos((xa + xb) / 2, ymid, bz) * Box(abs(xb - xa), ey1 - ey0, ear_h)).clean()
    top -= Pos(bx + out * (rb - 0.5), (y_lo + y_hi) / 2, bz) * Box(
        20, y_hi - y_lo, d.slit, align=(Align.MIN if out > 0 else Align.MAX, Align.CENTER, Align.CENTER))
    # the nut goes in from below, into a hex trap in the lower ear's underside, a flat towards the ring; the trap runs
    # on down through the ring's outside under it (the nut sits partly over the ring)
    xn = abs(ex - bx) - d.nut_af / 2
    ro = ring_r(angle, p, d)
    drop = max(0.0, (ro * ro - xn * xn) ** 0.5 - ear_h / 2 + 0.3) if xn < ro else 0.0
    top -= Pos(ex, ymid, bz - ear_h / 2 - drop - 0.01) * Rot(0, 0, 30) * nut_trap(d, d.nut_t + drop + 0.01)
    top -= Pos(ex, ymid, bz - ear_h / 2 - 1) * Cylinder(d.screw_clear_d / 2, ear_h + 2, align=MIN)
    top -= Pos(ex, ymid, bz + ear_h / 2) * countersunk(d, depth=d.screw_l + 0.5)
    top -= seat_cut(angle, p, d)
    return top


def plate_mid_y(side, p: Params = P, d: DriveParams = D):
    """|y| of the outer plate's mid-plane."""
    rp = d.insert_d / 2 + d.pillar_wall
    return (cap_screws(side, p, d)[0][1] - rp + housing_span(p)[1]) / 2


def fan_ear_top(p: Params = P, d: DriveParams = D):
    return p.axle_z + p.bearing.od / 2 + d.wall


def cap_screw_head(sx, sy, p: Params = P, d: DriveParams = D):
    """Cutter for a cap screw's head: the countersink, and above it the clearance open out through the outer face
    (the screws sit 2.3 mm in from it: a skin over the head would print as a 0.25 mm ceiling and break off)."""
    seat = p.axle_z + d.screw_l - d.insert_l  # so the M2x5 reaches the bottom of the insert (full thread engagement)
    r = d.csk_d / 2 + 0.05
    # one slot-shaped prism (a cylinder and a box meeting tangentially left a sliver along the seam)
    slot = extrude(make_hull(Circle(r).edges() + (Pos(0, 10) * Rectangle(2 * r, 20)).edges()).face(), 20)
    return Pos(sx, sy, seat) * (countersunk(d, depth=d.screw_l, up=0.01) + slot)


def nut_trap(d: DriveParams = D, depth=None):
    """Hexagonal pocket for an M2 nut, axis on +z from the origin."""
    from build123d import RegularPolygon
    return extrude(RegularPolygon(d.nut_af / 3 ** 0.5, 6), depth or d.nut_t)


def _fan_ear(cap, side, p, d):
    """Add the fan mount's ear to a (left-frame) cap: the hull of the front screw column and the tab, with a
    screw hole and a nut trap underneath; the front cap screw's counterbore stays open."""
    sx, sy = cap_screws(side, p, d)[0]
    tx, ty = d.fan_ear
    zt = fan_ear_top(p, d)
    rp = d.insert_d / 2 + d.pillar_wall
    edges = (Pos(sx, sy) * Circle(rp - 0.05)).edges() + (Pos(tx, ty) * Circle(d.fan_ear_r)).edges()
    ear = Pos(0, 0, zt - d.fan_ear_t) * extrude(make_hull(edges).face(), d.fan_ear_t)
    ear &= Pos(0, p.gear_y - p.stack.lip_gap, 0) * Box(200, 200, 200, align=(Align.CENTER, Align.MAX, Align.CENTER))  # clear of the gear
    cap = cap.fuse(ear).clean()
    cap -= Pos(tx, ty, zt - d.fan_recess) * Cylinder(d.fan_tab_r + 0.1, 5, align=MIN)
    cap -= Pos(tx, ty, zt - d.fan_ear_t - 1) * Cylinder(d.screw_clear_d / 2, d.fan_ear_t + 2, align=MIN)
    cap -= Pos(tx, ty, zt - d.fan_ear_t - 0.01) * nut_trap(d, d.nut_t + 0.01)
    if not solid(p):
        cap -= cap_screw_head(sx, sy, p, d)
    return cap


def block(side, p: Params = P, d: DriveParams = D):
    """The block of one side (side=+1 left, -1 right): (base, cap), or (block,) when Layout.blocks is "solid"."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    body, mxz = _left_block_solid(p, d, angle)
    body = _cut_left_bores(body, p, d, mxz)
    az = p.axle_z
    big = 200
    one = solid(p)
    if one:
        top, z_split = body, az
    else:
        base = body & Pos(0, 0, az) * Box(big, big, big, align=MAX)
        top = body & Pos(0, 0, az) * Box(big, big, big, align=MIN)
        # the cap's split face is relieved, so tightening it clamps the bearings and the level motor
        top -= Pos(0, 0, az) * Box(big, big, d.split_relief, align=MIN)
        z_split = az + d.split_relief
        # cap screws: countersunk through the cap, insert in the base below the split
        for sx, sy in cap_screws(side, p, d):
            base -= Pos(sx, sy, az) * Cylinder(d.insert_d / 2, d.cap_insert_depth, align=MAX)
            base -= Pos(sx, sy, az) * Cylinder(d.screw_clear_d / 2, d.screw_l, align=MAX)
            top -= cap_screw_head(sx, sy, p, d)
    # frame mounting boss with an insert, on the cap: blended into the motor seat's ring along the boss (hull of both),
    # from the boss's centre out; on the right from the ring's inboard end: that boss stands clear of the thin ring, so
    # its inboard side would start in mid-air as the cap prints (inboard face down). (On the left the boss stands in
    # its ring, and a blend inboard of it would meet the right motor.) Then a web joins it to the outer plate over its
    # full height (one piece with the plate), stopping short of a cap screw's head when that screw is next to the boss
    # (screwdriver access), and the boss itself goes on last. (The hull and the web are drawn 0.05 outside the boss's
    # sides, so the boss stands inside them: faces nearly tangent to it made the fuse fail, and a boss proud of them
    # showed as a strip; the order matters for the fuse too.)
    fx, fy, ftop = frame_boss(side, p, d)
    fz0 = motor_axis(angle, p)[1]
    rf = d.insert_d / 2 + d.wall
    wy1 = housing_span(p)[1]
    for sx, sy in cap_screws(side, p, d):
        if abs(sx - fx) < rf + d.csk_d / 2 + 0.3:
            wy1 = min(wy1, sy - d.csk_d / 2 - 0.3)
    wz0 = fz0 + 0.3  # (off the seat bore's seam, where the fuse fails)
    corners = [(fx - rf - 0.05, wz0, 0.01), (fx + rf + 0.05, wz0, 0.01),
               (fx - rf - 0.05, ftop - 0.01, 0.01), (fx + rf + 0.05, ftop - 0.01, 0.01)]
    ring = (*motor_axis(angle, p), ring_r(angle, p, d) - 0.05)  # (inside the ring: coincident faces break it)
    sy0, sy1 = seat_span(p, d)
    top = top.fuse(_belt([ring] + corners, sy0 if side < 0 else max(fy, sy0), min(wy1, sy1) - 0.02)
                   & Pos(0, 0, z_split) * Box(200, 200, 100, align=MIN)).clean()
    top = top.fuse(_belt(corners, fy, wy1 - 0.02)).clean()  # (`+` drops part of the web here)
    # (from the hull's floor up: a boss down at fz0 showed its bottom 0.3 under the hull)
    top += Pos(fx, fy, wz0 + 0.01) * Cylinder(rf, ftop - wz0 - 0.01, align=MIN)
    mx, mz = motor_axis(angle, p)
    top -= along_y(p.gears.tip_d(p.gears.pinion_z) / 2 + 0.4, seat_stop(p, d) - 0.01, 30, mx, mz)
    top -= along_y(p.gears.tip_d(p.gears.wheel_z) / 2 + 0.5, p.gear_y - p.stack.lip_gap, 40, 0, az)
    top -= Pos(fx, fy, ftop) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
    top -= Pos(fx, fy, ftop) * Cylinder(d.screw_clear_d / 2, d.frame_screw_depth, align=MAX)
    # re-cut the seat bore, the encoder slot, the cup's room and the cap screws' heads in case the boss or its blend
    # reached into them (the blend into the sleeve-sized rings filled the room's outer end, where the Ø6 cup's head turns)
    top -= seat_cut(angle, p, d)  # (as the first cut)
    top -= encoder_slot(angle, p, d)
    top -= cup_room(p)
    if not one:  # (the blend into the ring refilled a sliver of the rear cap screw's countersink)
        for sx, sy in cap_screws(side, p, d):
            top -= cap_screw_head(sx, sy, p, d)
    for a, out, y_max in ring_clamps(side, p, d):
        top = _ring_clamp(top, p, d, a, out, y_max)
    top = _fan_ear(top, side, p, d)
    # sockets for the basket's pegs, straight down into a flat landing on the plate's top
    for fs, fx, kind in d.basket_feet:
        if fs == side and kind == "peg":
            fy = plate_mid_y(side, p, d)
            r = d.socket_land_r
            probes = [(0.0, 0.0)] + [(r * cos(radians(a)), r * sin(radians(a))) for a in range(0, 360, 30)]
            zland = min((top & Pos(fx + dx, fy + dy, 0) * Box(0.1, 0.1, 100, align=MIN)).bounding_box().max.Z
                        for dx, dy in probes)
            top -= Pos(fx, fy, zland) * Box(2 * r, 20, 20, align=MIN)  # across the plate (a round one left skins)
            top -= Pos(fx, fy, zland - d.socket_depth) * Cylinder(d.socket_d / 2, 10, align=MIN)
    # the outer bearing's tube: one piece with the base (a full ring, not split)
    if one:
        top = top.fuse(tube(p, d)).clean()
    else:
        base = base.fuse(tube(p, d)).clean()
        top -= along_y(p.stack.tube_od / 2, p.housing_end_y - 0.02, 40, 0, az)  # (the tube's root overlaps the lip)
    parts = (top,) if one else (base, top)
    if side < 0:
        parts = tuple(mirror(x, Plane.XZ) for x in parts)
    tag = "L" if side > 0 else "R"
    for x, name in zip(parts, ("block",) if one else ("block_base", "block_cap")):
        x.label, x.color = f"{name}_{tag}", (0.9, 0.55, 0.2)
    return parts


def printed(p: Params = P, d: DriveParams = D):
    parts = {}
    for side in (1, -1):
        for part in block(side, p, d):
            parts[part.label] = part
    return parts


# ---- rotating parts -------------------------------------------------------------------------

@dataclass(frozen=True)
class WheelParams:
    rim: float = 0.8  # tire seat thickness
    web: float = 0.8  # disc joining rim and boss (on the gear side, glued to the gear face)
    boss_d: float = 4.0
    vents: int = 4  # through the web (3 x 1.5 trips a PrusaSlicer 2.9.4 crash in its support generator)
    vent_d: float = 1.5
    axle_fit: float = 0.04  # diametral, the wheel's bore on the axle: a light push, glued (0.08 was a little loose,
    # 0.15 loose; the thin resin bosses would split on a press fit)
    cup_axle_fit: float = 0.04  # the magnet cup's bore on the axle: a light push, glued (0.08 was a little loose)
    # the alternative cup for the Ø6x2 magnet (magnet_cup6_X; the Ø4x2 is the default, Params.magnet): the same
    # sleeve and the same gap to the chip; the blocks' room is drawn for it, so either cup fits
    alt_magnet: tuple = (6.0, 2.0)  # (d, t)
    magnet_fit: float = 0.02  # diametral, the magnet's pocket: a light press (0.06 at 2.5 s and 0.04 at 2.0 s let it
    # out too easily)
    # the brass gear (Gears.wheel_gear = "brass"): three M2x5 countersunk screws from the gear's inboard face (the
    # heads flush in 90 deg countersinks drilled in the brass: the gear runs 0.25 mm from the block) into nuts in
    # pockets that open on the wheel's outer face, on the circle halfway between the gear's bore and its tooth roots
    gear_screws: int = 3
    gear_screw_hole: float = 2.1  # in the wheel: the screw passes snug, so with the countersunk heads it centres the gear
    gear_boss_wall: float = 0.7  # the columns' wall round the nut pockets' corners


W = WheelParams()


def magnet_cup(side, p: Params = P, w: WheelParams = W, alt=False):
    """Cup holding the magnet on the inner axle end, with a long sleeve along the axle (in the housing's room) that
    ends in the cone bearing on the inner race only. alt=True: the alternative for the Ø6x2 magnet (magnet_cup6_X),
    with the same wall round it and its face at the same gap from the chip."""
    st = p.stack
    y0 = p.magnet_y
    md, mt = w.alt_magnet if alt else (p.magnet.d, p.magnet.t)
    y1 = y0 + mt + st.holder_wall
    cup = along_y(md / 2 + st.cup_wall, y0, y1)  # the same wall round either magnet
    inside = st.bearing_play + st.shoulder + 0.1  # from the race out past the shoulder's face
    cone = Pos(0, p.bearing_inner_y, 0) * Rot(90, 0, 0) * race_cone(inside, p.bearing_inner_y - inside - y1 + 0.01, p)
    cup += cone & along_y(st.sleeve_d / 2, y1 - 0.01, p.bearing_inner_y)
    cup -= along_y((md + w.magnet_fit) / 2, y0 - 1, y0 + mt)
    cup -= along_y((p.wheel.axle_d + w.cup_axle_fit) / 2, y0 + mt, p.bearing_inner_y + 1)
    cup = Pos(0, 0, p.axle_z) * cup
    return _side(cup, side, "magnet_cup6" if alt else "magnet_cup", (0.9, 0.9, 0.3))


def gear_screw_r(p: Params = P):
    """Radius of the brass gear's screw circle: halfway between its bore and its tooth roots."""
    g = p.gears
    return (p.wheel_hole_d / 2 + g.brass_root_d(g.wheel_z) / 2) / 2


def gear_screws(p: Params = P, w: "WheelParams" = None):
    """[(x, z, a)]: the brass gear's screws (robot frame, left; a: deg about the axle, from +x towards +z)."""
    w = w or W
    r = gear_screw_r(p)
    out = []
    for k in range(w.gear_screws):
        a = 90 + 360 * k / w.gear_screws
        out.append((r * cos(radians(a)), p.axle_z + r * sin(radians(a)), a))
    return out


def gear_nut_y(p: Params = P, w: "WheelParams" = None):
    """|y| of the brass gear screws' nut seats: web thick into the wheel's columns from the gear."""
    w = w or W
    return p.gear_y + p.gears.wheel_w + w.web


def brass_gear(side, p: Params = P, d: DriveParams = D):
    """The bought 36T as drilled (bought: in the layout, not printed): a disc to its tip circle, the bore opened to
    the wheel's hole, three countersunk M2 holes (the heads on its inboard face)."""
    g = p.gears
    gear = along_y(g.tip_d(g.wheel_z) / 2, p.gear_y, p.gear_y + g.wheel_w) - along_y(p.wheel_hole_d / 2, 0, 40)
    gear = Pos(0, 0, p.axle_z) * gear
    for x, z, _ in gear_screws(p):
        gear -= Pos(x, p.gear_y, z) * Rot(90, 0, 0) * countersunk(d, depth=5, up=1)
    if side < 0:
        gear = mirror(gear, Plane.XZ)
    gear.label, gear.color = f"wheel_gear_{'L' if side > 0 else 'R'}", (0.85, 0.7, 0.3)
    return gear


def wheel(side, p: Params = P, w: WheelParams = W):
    """The wheel: the drum that carries the tire, and the end web at the outer face with the axle boss and the cone
    onto the outer bearing's inner race. The gear is the brass one screwed on (Gears.wheel_gear = "brass": three
    columns inside the drum, one round each screw's nut pocket, run from the gear out to the end web, and their ends
    are all that touches the gear) or printed in one piece with the wheel ("printed", joined to the drum by a web on
    its outer face); either way its bore clears the tube."""
    from . import gears
    st, g, wh = p.stack, p.gears, p.wheel
    brass = g.wheel_gear == "brass"
    y_g1 = p.gear_y + g.wheel_w  # the gear's outer face
    y1, y_out = p.tire_outer_y, p.wheel_outer_y
    r, r_hole = p.hub_d / 2, p.wheel_hole_d / 2
    # web on the gear's outer face, inside the pinion's tip circle (the pinion's face is flush with the gear's)
    r_web = g.center_distance - g.tip_d(g.pinion_z) / 2 - 0.3
    y_web = y_out - st.end_web
    if brass:
        # the columns: a wall round each nut pocket (cut below), inside the drum (and so inside the pinion's tip
        # circle where they meet the gear), clear of the tube's bore
        part = None
        for x, z, _ in gear_screws(p, w):
            col = along_y(D.nut_af / 3 ** 0.5 + w.gear_boss_wall, y_g1, y_web + 0.01, x, z - p.axle_z)
            part = col if part is None else part + col
        part = part & along_y(r_web - 0.3 + 0.01, y_g1, y_web + 0.01) - along_y(r_hole, 0, 40)
    else:  # the printed gear (gears.py: axis +Z, outer face at z = 0) turned onto +y, and the web
        part = Pos(0, y_g1, 0) * Rot(-90, 0, 0) * gears.wheel_gear(p=p) + along_y(r_web, y_g1 - 0.01, y_g1 + w.web) - along_y(r_hole, 0, 40)
    # drum and the tire's channel (the seat between two flanges; the tire is stretched over the outer one), clear
    # of the gear face, hollow round the tube
    ya = y_g1 + wh.hub_relief
    drum = along_y(r, ya, y1) + along_y(r + wh.flange_h, ya, ya + wh.flange_in)
    drum += along_y(r + wh.flange_h, y1, y_out)
    drum -= along_y(r_web - 0.3, ya - 1, y_out + 1)
    part += drum
    # end web, the axle boss and the cone onto the outer bearing
    part += along_y(r_web - 0.29, y_web, y_out)
    # the axle boss runs in from the end web to just past the tube's end (inside the tube's bore radius), and ends
    # in the cone on the race: a long grip on the axle keeps the wheel square to it
    part += along_y(w.boss_d / 2, p.tube_end_y + st.lip_gap, y_out)
    inside = st.bearing_play + 0.1  # from the race to 0.1 past the tube's end
    cone = Pos(0, p.bearing_outer_y, 0) * Rot(-90, 0, 0) * race_cone(inside, y_web - p.bearing_outer_y - inside + 0.01, p)
    part += cone & along_y(w.boss_d / 2, p.bearing_outer_y - 1, y_out)
    if brass:
        # the nut pockets: each a hex (a flat facing out) from the nut's seat (gear_nut_y: the column's end is that
        # thick) out through the end web, open inwards to the bore (its inner flat would leave a 0.16 mm skin over
        # the bore). The drum is open at the gear's side, so it needs no vents
        y_seat = gear_nut_y(p, w)
        rs = gear_screw_r(p)
        for x, z, a in gear_screws(p, w):
            hexagon = Rot(0, 0, 30) * extrude(RegularPolygon(D.nut_af / 3 ** 0.5, 6), y_out + 1 - y_seat)
            slot = Pos(-(rs - r_hole + 0.5) / 2, 0, 0) * Box(rs - r_hole + 0.5, 2.0, y_out + 1 - y_seat, align=(Align.CENTER, Align.CENTER, Align.MIN))
            part -= Pos(x, y_seat, z - p.axle_z) * Rot(-90, 0, 0) * Rot(0, 0, -a) * (hexagon + slot)
            part -= along_y(w.gear_screw_hole / 2, y_g1 - 1, y_seat + 0.01, x, z - p.axle_z)
    else:
        # vents through the end web: printed end web down, the drum would otherwise be a cup facing the resin film
        rv = (w.boss_d / 2 + r_web - 0.3) / 2
        for k in range(w.vents):
            a = radians(360 * k / w.vents + 45)
            part -= along_y(w.vent_d / 2, y_web - 1, y_out + 1, rv * cos(a), rv * sin(a))
    part -= along_y((wh.axle_d + w.axle_fit) / 2, 0, 40)
    return _side(Pos(0, 0, p.axle_z) * part, side, "wheel", (0.95, 0.95, 0.95))


def _side(part, side, name, color):
    if side < 0:
        part = mirror(part, Plane.XZ)
    part.label = f"{name}_{'L' if side > 0 else 'R'}"
    part.color = color
    return part


def rotating(p: Params = P, w: WheelParams = W):
    parts = {}
    for side in (1, -1):
        for fn in (magnet_cup, wheel):
            part = fn(side, p, w)
            parts[part.label] = part
    return parts


def all_parts(p: Params = P):
    return {**printed(p), **rotating(p)}
