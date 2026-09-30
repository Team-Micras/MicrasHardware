"""Printed drivetrain parts: bearing block base + cap per side, eccentric motor sleeves, magnet cups.

Each side's block is split at the axle plane (z = axle_z). The base is screwed to the board through
the two countersunk holes; the cap clamps the bearings (and, on the left, the level motor's sleeve)
with two screws into inserts in the base. The right cap also carries the clamp ring of the raised motor.
"""

from dataclasses import dataclass
from math import atan2, cos, degrees, radians, sin

from build123d import (Align, Box, Circle, Cone, Cylinder, Line, Plane, Pos, Rectangle, Rot, ThreePointArc, Wire,
                       extrude, make_face, make_hull, mirror, offset)

from .layout import board_boxes
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class DriveParams:
    wall: float = 1.2  # around bearing and sleeve bores
    bearing_fit: float = 0.1  # diametral clearance on the bearing OD (the split closes by split_relief)
    sleeve_wall: float = 0.8  # thinnest wall of the eccentric sleeve
    sleeve_fit: float = 0.1  # diametral clearance sleeve in seat (the split closes by split_relief)
    pillar_wall: float = 1.0  # around the glued cap-screw inserts
    plate_t: float = 2.4  # outer plate skin where it is pocketed from the inboard side
    pocket_rim: float = 1.0  # solid rim around the pockets, along the plate's outline
    pocket_margin: float = 0.8  # pockets stay this far from the rings, screw columns and housing web
    split_relief: float = 0.15  # taken off the cap's split face: tightening the cap clamps bearings and sleeve
    motor_fit: float = 0.05  # diametral clearance motor in sleeve
    sleeve_lip: float = 0.6  # front lip that stops the motor axially
    seat_min_y: float = 7.6  # seats stay outboard of the encoder daughterboards
    pad_inset: float = 0.3  # stay inside the silkscreen contact outline
    lift: float = 1.8  # everything except the pads starts this far above the board top
    pad_h: float = 3.0  # contact plate thickness
    insert_d: float = 3.35  # threaded insert hole in resin (M2x2 OD 3.2, glued in)
    insert_l: float = 2.0
    screw_clear_d: float = 2.2
    screw_l: float = 5.0
    csk_d: float = 4.0  # countersink for the M2 flat heads in the caps
    cap_screw_y: float = 16.0  # |y| of the cap screws, at most (the head stays 0.3 clear of the gear cut-out)
    cap_screw_front_x: float = 5.2  # clear of the fan mount feet; the rear screw's x is found per side (cap_screws)
    frame_screw_depth: float = 3.8  # M2x5 through a 1.2 frame floor
    frame_boss_top_left: float = 21.8  # keeps the screw tip 0.5 above the left seat bore
    tray_bottom_z: float = 28.8  # frame tray underside, sits on the right boss
    slit: float = 0.8  # clamp slit in the raised motor's ring
    ear: float = 2.2  # clamp ear thickness either side of the slit (M2x5 then engages 2 mm)
    notch_d: float = 1.0  # spanner notches on the sleeve rim (width)
    notch_depth: float = 0.6  # radial, on the sleeve's thick side


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


def sleeve_od(p=P, d=D):
    return p.motor.d + d.motor_fit + 2 * (d.sleeve_wall + p.layout.eccentricity)


def seat_d(p=P, d=D):
    if p.layout.backlash_mode == "eccentric":
        return sleeve_od(p, d) + d.sleeve_fit
    return p.motor.d + d.motor_fit


def seat_span(p=P, d=D):
    """|y| extent of a motor seat (inner, outer)."""
    return d.seat_min_y, p.motor_front_y + d.sleeve_lip


def motor_axis(angle, p=P):
    """Motor axis (x, z) for a motor at `angle` around the wheel axle."""
    a = radians(angle)
    cd = p.gears.center_distance
    return cd * cos(a), p.axle_z + cd * sin(a)


def seat_axis(angle, p=P):
    """Seat axis: the motor axis shifted by the sleeve eccentricity (perpendicular to the centre line),
    so the neutral sleeve orientation puts the motor at the nominal centre distance."""
    mx, mz = motor_axis(angle, p)
    if p.layout.backlash_mode != "eccentric":
        return mx, mz
    a, e = radians(angle), p.layout.eccentricity
    return mx + sin(a) * e, mz - cos(a) * e


def housing_span(p=P):
    """|y| extent of the bearing housing (shoulder to lip)."""
    return p.shoulder_y, p.bearing_outer_y + p.stack.lip


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
    rs = seat_d(p, d) / 2 + d.wall
    mx, mz = seat_axis(motor_angle, p)

    # contact plate on the silkscreen zone (the board screws go into it from below)
    body = Pos(0, 0, b.top_z) * extrude(contact_zone(p, d.pad_inset), d.pad_h)
    # bearing housing and motor seat as one body: the hull of both rings over the length where they
    # overlap; beyond it each continues as its own ring, with flat, solid end faces
    ring_h, ring_s = (0, az, rh), (mx, mz, rs)
    body += along_y(rh, hy0, hy1, 0, az) + along_y(rs, sy0, sy1, mx, mz)
    body += _belt((ring_h, ring_s), max(hy0, sy0), min(hy1, sy1))
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
        # the boss reaches up into the seat ring when the ring is raised above it
        top = az + rh
        near = abs(sx - mx) - rp
        if mz > top and near < rs:
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
            Pos(sx, 50) * Rectangle(2 * rp, 200) for sx, _ in screws]:
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


def _cut_left_bores(body, p: Params, d: DriveParams, motor_angle):
    b = p.board
    az = p.axle_z
    hy0, hy1 = housing_span(p)
    sy0, sy1 = seat_span(p, d)
    sx, sz = seat_axis(motor_angle, p)
    mx, mz = motor_axis(motor_angle, p)
    st = p.stack
    # bearing bore, shoulder and lip bores
    # two bearing bores with a ridge between them (its bore clears the inner races, like the shoulder)
    rb = (p.bearing.od + d.bearing_fit) / 2
    body -= along_y(rb, p.bearing_inner_y, p.bearing_inner_y + p.bearing.w, 0, az)
    body -= along_y(rb, p.bearing_outer_y - p.bearing.w, p.bearing_outer_y, 0, az)
    body -= along_y(st.shoulder_id / 2, hy0 - 1, hy1 + 1, 0, az)
    # room for the magnet cup (rotating)
    body -= along_y(p.magnet.d / 2 + st.holder_wall + st.holder_gap, 0, hy0, 0, az)
    # motor seat bore (through; the lip belongs to the sleeve)
    body -= along_y(seat_d(p, d) / 2, sy0 - 1, sy1 + 1, sx, sz)
    # clearance for the pinion and the motor can beyond the seat
    body -= along_y(p.gears.tip_d(p.gears.pinion_z) / 2 + 0.4, sy1, 30, mx, mz)
    body -= along_y(p.motor.d / 2 + 0.4, -30, sy0, mx, mz)
    # wheel gear and tire clearance
    body -= along_y(p.gears.tip_d(p.gears.wheel_z) / 2 + 0.5, p.gear_y - p.stack.lip_gap, 40, 0, az)
    # board screws: insert at the pad bottom, clearance above
    for hx, hy in (b.front_hole, b.rear_hole):
        body -= Pos(hx, hy, b.top_z) * Cylinder(d.insert_d / 2, d.insert_l, align=MIN)
        body -= Pos(hx, hy, b.top_z) * Cylinder(d.screw_clear_d / 2, d.screw_l - b.thickness + 0.8, align=MIN)
    return body


def cap_screws(side, p: Params = P, d: DriveParams = D):
    """(x, |y|) of the two cap screws on one side. The front one sits in front of the housing; the rear one
    is moved back until its insert clears the motor seat bore and its screwdriver clears the pinion."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    sx, sz = seat_axis(angle, p)
    mx, mz = motor_axis(angle, p)
    rs = seat_d(p, d) / 2 + 0.3
    r_ins = d.insert_d / 2
    r_pin = p.gears.tip_d(p.gears.pinion_z) / 2 + 0.3
    az = p.axle_z
    x = -d.cap_screw_front_x
    for _ in range(300):
        # insert (az - insert_l .. az) vs the seat bore circle, in the xz plane
        dz = max(0.0, abs(sz - (az - d.insert_l / 2)) - d.insert_l / 2)
        clear_seat = ((x - sx) ** 2 + dz ** 2) ** 0.5 >= rs + r_ins
        # a Ø2.6 driver straight down onto the head vs the pinion above it
        clear_pinion = mz < az or abs(x - mx) >= r_pin + 1.3
        # ... and vs the seat ring (for z above the head), where the ring reaches over the screw's y
        z_head = az + d.screw_l - d.insert_l
        ring_r = seat_d(p, d) / 2 + d.wall
        clear_ring = ((x - sx) ** 2 + (max(z_head, sz) - sz) ** 2) ** 0.5 >= ring_r + 1.5
        if clear_seat and clear_pinion and clear_ring:
            break
        x -= 0.1
    y = min(d.cap_screw_y, p.gear_y - p.stack.lip_gap - d.csk_d / 2 - 0.3)  # head clear of the gear cut-out
    return ((d.cap_screw_front_x, y), (round(x, 2), y))


def frame_boss(side, p: Params = P, d: DriveParams = D):
    """(x, |y|, top z) of the frame mounting boss on each cap."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    sx, sz = seat_axis(angle, p)
    if side > 0:  # on top of the level motor's seat
        return sx, 11.0, d.frame_boss_top_left
    # behind the raised motor's ring, far enough out that the hole misses the bore
    return sx - (seat_d(p, d) / 2 + d.insert_d / 2 + 0.6), 11.0, d.tray_bottom_z


def _ring_clamp(cap, p, d, angle):
    """Slit the raised motor's ring on its front side and add a vertical clamp screw across the slit."""
    sx, sz = seat_axis(angle, p)
    sy0, sy1 = seat_span(p, d)
    rs = seat_d(p, d) / 2
    ro = rs + d.wall
    # the ear stops short of the front cap screw, so a screwdriver reaches that screw past it
    ey1 = min(sy1, cap_screws(-1, p, d)[0][1] - 1.5)
    ymid = (sy0 + ey1) / 2
    ex = sx + ro + d.insert_d / 2  # screw axis, just outside the ring
    ear_h = 2 * d.ear + d.slit
    cap += Pos((sx + rs + ex + d.insert_d / 2 + d.wall) / 2, ymid, sz) * Box(
        ex + d.insert_d / 2 + d.wall - sx - rs, ey1 - sy0, ear_h)
    hy0, hy1 = housing_span(p)
    y_lo, y_hi = min(sy0, hy0) - 1, max(sy1, hy1) + 1  # through the whole ring and its tapers
    cap -= Pos(sx + rs - 0.5, (y_lo + y_hi) / 2, sz) * Box(20, y_hi - y_lo, d.slit, align=(Align.MIN, Align.CENTER, Align.CENTER))
    top = sz + ear_h / 2
    cap -= Pos(ex, ymid, sz - d.slit / 2) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
    cap -= Pos(ex, ymid, top) * countersunk(d, depth=d.screw_l + 0.5)
    cap -= along_y(rs, sy0 - 1, sy1 + 1, sx, sz)
    return cap


def block(side, p: Params = P, d: DriveParams = D):
    """(base, cap) for one side; side=+1 left, -1 right."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    body, mxz = _left_block_solid(p, d, angle)
    body = _cut_left_bores(body, p, d, mxz)
    az = p.axle_z
    big = 200
    base = body & Pos(0, 0, az) * Box(big, big, big, align=MAX)
    cap = body & Pos(0, 0, az) * Box(big, big, big, align=MIN)
    # the cap's split face is relieved, so tightening it clamps the bearings and the level motor's sleeve
    cap -= Pos(0, 0, az) * Box(big, big, d.split_relief, align=MIN)
    # cap screws: countersunk through the cap, insert in the base below the split
    for sx, sy in cap_screws(side, p, d):
        # head seat chosen so the M2x5 reaches the bottom of the insert (full thread engagement)
        seat = az + d.screw_l - d.insert_l
        base -= Pos(sx, sy, az) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
        base -= Pos(sx, sy, az) * Cylinder(d.screw_clear_d / 2, d.screw_l, align=MAX)
        cap -= Pos(sx, sy, seat) * countersunk(d, depth=d.screw_l)
    # frame mounting boss with an insert, on the cap
    fx, fy, ftop = frame_boss(side, p, d)
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    fz0 = seat_axis(angle, p)[1]
    rf = d.insert_d / 2 + d.wall
    cap += Pos(fx, fy, fz0) * Cylinder(rf, ftop - fz0, align=MIN)
    # ... joined to the outer plate by a web over its full height (one piece with the plate); the web
    # stops short of a cap screw's head when that screw is next to the boss (screwdriver access)
    wy1 = housing_span(p)[1]
    for sx, sy in cap_screws(side, p, d):
        if abs(sx - fx) < rf + d.csk_d / 2 + 0.3:
            wy1 = min(wy1, sy - d.csk_d / 2 - 0.3)
    wz0 = fz0 + 0.3  # (off the seat bore's seam, where the fuse fails)
    web = _belt([(fx - rf + 0.01, wz0, 0.01), (fx + rf - 0.01, wz0, 0.01),
                 (fx - rf + 0.01, ftop - 0.01, 0.01), (fx + rf - 0.01, ftop - 0.01, 0.01)], fy, wy1 - 0.02)
    cap = cap.fuse(web).clean()  # (`+` drops part of the web here)
    # ... and blended into the motor seat's ring along the boss (hull of both), from the boss's centre out
    ring = (*seat_axis(angle, p), seat_d(p, d) / 2 + d.wall)
    sy0, sy1 = seat_span(p, d)
    cap = cap.fuse(_belt([ring, (fx - rf + 0.01, wz0, 0.01), (fx + rf - 0.01, wz0, 0.01),
                          (fx - rf + 0.01, ftop - 0.01, 0.01), (fx + rf - 0.01, ftop - 0.01, 0.01)],
                         max(fy, sy0), min(wy1, sy1) - 0.02)
                   & Pos(0, 0, az + d.split_relief) * Box(200, 200, 100, align=MIN)).clean()
    mx, mz = motor_axis(angle, p)
    cap -= along_y(p.gears.tip_d(p.gears.pinion_z) / 2 + 0.4, seat_span(p, d)[1], 30, mx, mz)
    cap -= along_y(p.gears.tip_d(p.gears.wheel_z) / 2 + 0.5, p.gear_y - p.stack.lip_gap, 40, 0, az)
    cap -= Pos(fx, fy, ftop) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
    cap -= Pos(fx, fy, ftop) * Cylinder(d.screw_clear_d / 2, d.frame_screw_depth, align=MAX)
    # re-cut the seat bore in case the boss reached into it
    cap -= along_y(seat_d(p, d) / 2, *seat_span(p, d), *seat_axis(angle, p))
    if side < 0 and p.layout.backlash_mode == "eccentric":
        cap = _ring_clamp(cap, p, d, angle)
    if side < 0:
        base = mirror(base, Plane.XZ)
        cap = mirror(cap, Plane.XZ)
    tag = "L" if side > 0 else "R"
    base.label, cap.label = f"block_base_{tag}", f"block_cap_{tag}"
    base.color = cap.color = (0.9, 0.55, 0.2)
    return base, cap


def sleeve(side, p: Params = P, d: DriveParams = D):
    """Eccentric motor sleeve, placed at its nominal orientation (eccentricity pointing at the axle)."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    mx, mz = motor_axis(angle, p)
    sx, sz = seat_axis(angle, p)
    sy0, sy1 = seat_span(p, d)
    s = along_y(sleeve_od(p, d) / 2, sy0, sy1, sx, sz)
    # the bore is offset by e: turning the sleeve moves the motor by up to +-e along the centre line
    s -= along_y((p.motor.d + d.motor_fit) / 2, sy0 - 1, sy1 - d.sleeve_lip, mx, mz)
    s -= along_y(p.motor.boss_d / 2 + 0.3, sy0, sy1 + 1, mx, mz)
    # two spanner notches in the inner rim, to turn the sleeve with tweezers: on the sleeve's thick side
    # (away from the bore offset), 60 deg either side of it
    r_n = sleeve_od(p, d) / 2 - d.notch_depth / 2
    thick = degrees(atan2(-(mz - sz), -(mx - sx)))
    for k in (-60, 60):
        a = radians(thick + k)
        s -= Pos(sx + r_n * cos(a), sy0, sz + r_n * sin(a)) * Rot(0, -(thick + k), 0) * Box(
            d.notch_depth, 2 * d.notch_d, d.notch_d)
    if side < 0:
        s = mirror(s, Plane.XZ)
    s.label = f"sleeve_{'L' if side > 0 else 'R'}"
    s.color = (0.3, 0.7, 0.9)
    return s


def printed(p: Params = P, d: DriveParams = D):
    parts = {}
    for side in (1, -1):
        base, cap = block(side, p, d)
        parts[base.label] = base
        parts[cap.label] = cap
        if p.layout.backlash_mode == "eccentric":
            sl = sleeve(side, p, d)
            parts[sl.label] = sl
    return parts


# ---- rotating parts -------------------------------------------------------------------------

@dataclass(frozen=True)
class WheelParams:
    rim: float = 0.8  # tire seat thickness
    web: float = 0.8  # disc joining rim and boss (on the gear side, glued to the gear face)
    boss_d: float = 4.0
    lip_h: float = 0.3  # small outer lip that keeps the tire from walking off
    lip_w: float = 0.5
    axle_fit: float = 0.03  # diametral clearance on the axle (magnet cup and hub are glued; the thin resin
    # bosses would split on a press fit)


W = WheelParams()


def magnet_cup(side, p: Params = P, w: WheelParams = W):
    """Cup holding the magnet on the inner axle end; its boss bears on the inner race only."""
    st = p.stack
    y0 = p.magnet_y
    y1 = y0 + p.magnet.t + st.holder_wall
    od = p.magnet.d + 2 * 0.5
    cup = along_y(od / 2, y0, y1) + along_y(st.boss_d / 2, y1, p.bearing_inner_y)
    cup -= along_y(p.magnet.d / 2 + 0.02, y0 - 1, y0 + p.magnet.t)
    cup -= along_y((p.wheel.axle_d + w.axle_fit) / 2, y0 + p.magnet.t, p.bearing_inner_y + 1)
    cup = Pos(0, 0, p.axle_z) * cup
    return _side(cup, side, "magnet_cup", (0.9, 0.9, 0.3))


def race_spacer(side, p: Params = P, w: WheelParams = W):
    """Spacer between the outer bearing's inner race and the wheel gear."""
    s = along_y(p.stack.boss_d / 2, p.bearing_outer_y, p.gear_y)
    s -= along_y((p.wheel.axle_d + 0.05) / 2, 0, 40)
    return _side(Pos(0, 0, p.axle_z) * s, side, "race_spacer", (0.9, 0.9, 0.3))


def wheel_hub(side, p: Params = P, w: WheelParams = W):
    """Tire seat, glued to the outer face of the wheel gear."""
    y0 = p.gear_y + p.gears.wheel_w
    y1 = p.tire_outer_y
    r = p.hub_d / 2
    hub = along_y(r, y0, y1) - along_y(r - w.rim, y0 + w.web, y1 + 1)
    # lip just outboard of the tire, an open ring (a solid disc would close the drum)
    hub += along_y(r + w.lip_h, y1, y1 + w.lip_w) - along_y(r - w.rim, y1 - 1, y1 + w.lip_w + 1)
    hub += along_y(w.boss_d / 2, y0, y1)
    hub -= along_y((p.wheel.axle_d + w.axle_fit) / 2, 0, 40)
    return _side(Pos(0, 0, p.axle_z) * hub, side, "wheel_hub", (0.95, 0.95, 0.95))


def _side(part, side, name, color):
    if side < 0:
        part = mirror(part, Plane.XZ)
    part.label = f"{name}_{'L' if side > 0 else 'R'}"
    part.color = color
    return part


def rotating(p: Params = P, w: WheelParams = W):
    parts = {}
    for side in (1, -1):
        for fn in (magnet_cup, race_spacer, wheel_hub):
            part = fn(side, p, w)
            parts[part.label] = part
    return parts


def all_parts(p: Params = P):
    return {**printed(p), **rotating(p)}
