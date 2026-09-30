"""Suction fan: closed centrifugal impeller and its mount.

The impeller's flat front shroud runs just above the component-free Ø27 silkscreen ring around the
board hole, which acts as the inlet seal. The mount stands on three feet on free board spots just
outside the ring and holds the motor vertically, shaft down; the top frame presses it onto the board.
"""

from dataclasses import dataclass
from math import cos, radians, sin

from build123d import Align, Axis, Box, Cylinder, Plane, Polyline, Pos, Rot, make_face, revolve

from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FanParams:
    # impeller
    d2: float = 26.4  # outer diameter (max: nearest component 13.5 mm from the fan centre, less 0.3)
    eye_d: float = 15.5  # inlet eye (board hole is 15.0)
    blades: int = 10
    blade_t: float = 0.65
    blade_angle: float = 90.0  # outlet angle from tangent: 90 = radial (with skirt), ~60 = backward curved
    h_eye: float = 5.5  # blade height at the eye
    h_tip: float = 4.0  # blade height at the tip
    shroud_t: float = 0.6  # front shroud (bottom)
    back_t: float = 0.8  # backplate (top)
    seal_gap: float = 0.3  # front shroud to board top
    hub_d: float = 4.5
    hub_above: float = 1.5  # hub boss above the backplate
    nose_d: float = 7.0  # flow-turning cone under the hub
    bore_d: float = 0.9  # printed undersize, ream to 0.97-0.98
    # mount
    feet: tuple = ((130.0, 15.5), (230.0, 15.5), (330.0, 14.8))  # (angle from +x, radius) on free spots
    foot_d: float = 2.5
    post_d: float = 2.0
    plate_t: float = 1.2
    plate_gap: float = 0.4  # hub top to plate
    collar_wall: float = 1.2
    collar_h: float = 7.0
    motor_fit: float = 0.05


F = FanParams()


def centre(p: Params = P):
    return p.board.fan_hole_x, 0.0


def heights(p: Params = P, f: FanParams = F):
    """Key z levels of the fan stack."""
    z_shroud = p.board.top_z + f.seal_gap
    z_blades = z_shroud + f.shroud_t
    z_back_tip = z_blades + f.h_tip
    z_back_eye = z_blades + f.h_eye
    z_hub_top = z_back_eye + f.back_t + f.hub_above
    z_plate = z_hub_top + f.plate_gap
    z_motor = z_plate + f.plate_t  # motor front face
    return dict(shroud=z_shroud, blades=z_blades, back_tip=z_back_tip, back_eye=z_back_eye,
                hub_top=z_hub_top, plate=z_plate, motor=z_motor,
                shaft_end=z_motor - p.motor.shaft_l, motor_top=z_motor + p.motor.body_l + p.motor.rear_l)


def _revolved(points):
    """Solid of revolution about the fan axis from an (r, z) polygon."""
    return revolve(Plane.XZ * make_face(Polyline(*points, close=True)), Axis.Z)


def impeller(p: Params = P, f: FanParams = F):
    h = heights(p, f)
    r2, r1, rh = f.d2 / 2, f.eye_d / 2, f.hub_d / 2
    zs, zb, zt, ze = h["shroud"], h["blades"], h["back_tip"], h["back_eye"]
    # front shroud (flat ring with the eye) and conical backplate of constant thickness
    body = _revolved([(r1, zs), (r2, zs), (r2, zb), (r1, zb)])
    body += _revolved([(0, ze), (r1, ze), (r2, zt), (r2, zt + f.back_t), (r1, ze + f.back_t), (0, ze + f.back_t)])
    # blades, trimmed to the flow channel between shroud and backplate
    channel = _revolved([(rh, zb), (r2, zb), (r2, zt), (r1, ze), (rh, ze)])
    blade_len = r2 - rh
    blades = None
    for i in range(f.blades):
        blade = Pos(rh + blade_len / 2, 0, zb) * Box(blade_len, f.blade_t, ze - zb, align=(Align.CENTER, Align.CENTER, Align.MIN))
        if f.blade_angle != 90:
            # straight blade leaning back: outlet angle measured from the tangent
            blade = Pos(rh, 0, 0) * Rot(0, 0, -(90 - f.blade_angle)) * Pos(-rh, 0, 0) * blade
        blade = Rot(0, 0, 360 * i / f.blades) * blade
        blades = blade if blades is None else blades + blade
    body += blades & channel
    # nose cone turning the inflow, hub boss, shaft bore
    body += _revolved([(0, zb + 0.8), (rh * 0.6, zb + 0.8), (f.nose_d / 2, ze), (0, ze)])
    body += Pos(0, 0, ze) * Cylinder(rh, h["hub_top"] - ze, align=MIN)
    body -= Pos(0, 0, h["shaft_end"] - 0.3) * Cylinder(f.bore_d / 2, 30, align=MIN)
    x, y = centre(p)
    body = Pos(x, y, 0) * body
    body.label, body.color = "impeller", (0.95, 0.4, 0.4)
    return body


def mount(p: Params = P, f: FanParams = F):
    h = heights(p, f)
    top = p.board.top_z
    cx, cy = centre(p)
    rc = (p.motor.d + f.motor_fit) / 2 + f.collar_wall
    plate_r = max(r for _, r in f.feet)
    body = Pos(0, 0, h["plate"]) * Cylinder(rc, f.plate_t + f.collar_h, align=MIN)
    for ang, r in f.feet:
        fx, fy = r * cos(radians(ang)), r * sin(radians(ang))
        body += Pos(fx, fy, top) * Cylinder(f.foot_d / 2, h["plate"] - top + f.plate_t, align=MIN)
        # arm from the foot post to the collar
        arm = Box(r, f.post_d, f.plate_t, align=(Align.MIN, Align.CENTER, Align.MIN))
        body += Pos(0, 0, h["plate"]) * Rot(0, 0, ang) * arm
    # motor bore, boss hole, shaft hole
    body -= Pos(0, 0, h["motor"]) * Cylinder((p.motor.d + f.motor_fit) / 2, 30, align=MIN)
    body -= Pos(0, 0, h["plate"] - 1) * Cylinder(p.motor.boss_d / 2 + 0.3, 5, align=MIN)
    # clearance around the spinning impeller
    body -= Pos(0, 0, top) * Cylinder(f.d2 / 2 + 0.5, h["plate"] - top, align=MIN)
    body = Pos(cx, cy, 0) * body
    body.label, body.color = "fan_mount", (0.9, 0.55, 0.2)
    return body


def fan_motor(p: Params = P, f: FanParams = F):
    from .purchased import motor
    h = heights(p, f)
    cx, cy = centre(p)
    m = Pos(cx, cy, h["motor"]) * Rot(180, 0, 0) * motor(p.motor)  # shaft down
    m.label = "fan_motor"
    return m


def parts(p: Params = P, f: FanParams = F):
    return {"impeller": impeller(p, f), "fan_mount": mount(p, f), "fan_motor": fan_motor(p, f)}
