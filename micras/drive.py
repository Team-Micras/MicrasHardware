"""Printed drivetrain parts: bearing block base + cap per side, eccentric motor sleeves, magnet cups.

Each side's block is split at the axle plane (z = axle_z). The base is screwed to the board through
the two countersunk holes; the cap clamps the bearings (and, on the left, the level motor's sleeve)
with two screws into inserts in the base. The right cap also carries the clamp ring of the raised motor.
"""

from dataclasses import dataclass
from math import atan2, cos, degrees, radians, sin

from build123d import (Align, Box, Cylinder, Line, Plane, Pos, Rot, ThreePointArc, Wire, extrude,
                       make_face, mirror)

from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class DriveParams:
    wall: float = 1.2  # around bearing and sleeve bores
    bearing_fit: float = 0.0  # diametral clearance on the bearing OD (split housing clamps it)
    sleeve_wall: float = 0.8  # thinnest wall of the eccentric sleeve
    sleeve_fit: float = 0.1  # diametral clearance sleeve in seat (clamped)
    motor_fit: float = 0.05  # diametral clearance motor in sleeve
    sleeve_lip: float = 0.6  # front lip that stops the motor axially
    seat_min_y: float = 7.6  # seats stay outboard of the encoder daughterboards
    pad_inset: float = 0.3  # stay inside the silkscreen contact outline
    lift: float = 1.8  # everything except the pads starts this far above the board top
    pad_h: float = 3.0  # contact plate thickness
    insert_d: float = 3.2  # threaded insert hole (resin: glue in)
    insert_l: float = 2.0
    screw_clear_d: float = 2.2
    screw_l: float = 5.0
    csk_d: float = 4.0  # countersink for the M2 flat heads in the caps
    cap_screws: tuple = ((5.6, 16.0), (-5.6, 16.0))  # (x, |y|) cap screws, same for both sides
    ring_clamp_screw: bool = True


D = DriveParams()


# ---- helpers --------------------------------------------------------------------------------

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
    # bearing housing
    body += along_y(rh, hy0, hy1, 0, az)
    # web from the housing down towards the pads
    lz = b.top_z + d.lift
    body += Pos(0, (hy0 + hy1) / 2, lz) * Box(2 * rh, hy1 - hy0, az - lz, align=MIN)
    # motor seat and a web joining it to the housing
    body += along_y(rs, sy0, sy1, mx, mz)
    web_len = ((mx) ** 2 + (mz - az) ** 2) ** 0.5
    web = Pos(0, 0, 0) * Box(web_len, sy1 - max(sy0, hy0), 2 * min(rh, rs) * 0.8,
                             align=(Align.MIN, Align.CENTER, Align.CENTER))
    web = Pos(0, (sy1 + max(sy0, hy0)) / 2, az) * Rot(0, -degrees(atan2(mz - az, mx)), 0) * web
    body += web
    # cap screw bosses (full height from pads to cap top)
    for sx, sy in d.cap_screws:
        body += Pos(sx, sy, lz) * Cylinder(d.insert_d / 2 + d.wall, az + rh - lz, align=MIN)
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
    body -= along_y((p.bearing.od + d.bearing_fit) / 2, p.bearing_inner_y, p.bearing_outer_y, 0, az)
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


def block(side, p: Params = P, d: DriveParams = D):
    """(base, cap) for one side; side=+1 left, -1 right."""
    angle = p.layout.motor_angle_left if side > 0 else p.layout.motor_angle_right
    body, mxz = _left_block_solid(p, d, angle)
    body = _cut_left_bores(body, p, d, mxz)
    az = p.axle_z
    big = 200
    base = body & Pos(0, 0, az) * Box(big, big, big, align=MAX)
    cap = body & Pos(0, 0, az) * Box(big, big, big, align=MIN)
    # cap screws: countersunk through the cap, insert in the base below the split
    for sx, sy in d.cap_screws:
        hole = Pos(sx, sy, az - d.screw_l + 3.0) * Cylinder(d.screw_clear_d / 2, 20, align=MIN)
        base -= Pos(sx, sy, az) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
        base -= hole
        cap -= hole
        cap_top = cap.bounding_box().max.Z
        cap -= Pos(sx, sy, cap_top) * Cylinder(d.csk_d / 2, (d.csk_d - d.screw_clear_d) / 2, align=MAX)
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
    axle_fit: float = -0.02  # diametral: negative = press fit on the axle


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
    hub += along_y(r + w.lip_h, y1 - w.lip_w, y1)
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
