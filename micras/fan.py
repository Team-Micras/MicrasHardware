"""Suction fan: a closed radial impeller running just above the board, and its mount.

Sized for the measured motor (18k rpm no-load at 12 V, so speed-limited), following the fan study: the
impeller is as large as the layout allows (Ø26.4: nearest part 13.5 mm from the hole centre; a larger,
raised impeller would hit the encoder daughterboards and the raised drive motor), with a small eye (11),
12 radial blades and a flat 3 mm channel.

- Impeller (resin): flat front shroud 0.3 mm above the component-free Ø27 silkscreen ring (face seal),
  plus a short neck that dips into the board hole with a small radial gap. Hub pressed/glued on the shaft.
- Mount (resin): symmetric, standing on two feet on free board spots behind the ring. The collar's top is
  slotted and tapered: the body's airbox presses on it, which holds the mount down and clamps the motor
  like a collet (no screws).
"""

from dataclasses import dataclass
from math import cos, radians, sin

from build123d import Align, Axis, Box, Cone, Cylinder, Plane, Polyline, Pos, Rot, extrude, make_face, revolve

from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FanParams:
    # impeller
    d2: float = 26.4  # nearest component 13.5 mm from the fan centre, less 0.3
    eye_d: float = 11.0
    blades: int = 12
    blade_t: float = 0.55
    blade_angle: float = 90.0  # outlet angle from tangent: 90 = radial (best near shut-off, with skirt)
    h_eye: float = 3.0  # blade height at the eye
    h_tip: float = 3.0  # blade height at the tip (flat backplate)
    shroud_t: float = 0.6  # front shroud (bottom)
    back_t: float = 0.8  # backplate (top)
    shroud_z: float = 0.3  # front shroud underside above the board top (face seal on the Ø27 ring)
    neck_id: float = 13.4  # neck under the shroud: OD 14.4 in the Ø15 board hole
    neck_t: float = 0.5
    neck_l: float = 1.2  # dips 0.9 mm into the board hole
    hub_d: float = 4.5
    hub_above: float = 1.5  # hub boss above the backplate
    nose_d: float = 6.0  # flow-turning cone under the hub
    bore_d: float = 0.9  # printed undersize, ream to 0.97-0.98
    # mount
    feet: tuple = ((130.0, 15.5), (230.0, 15.5))  # (angle from +x, radius): symmetric, on free board spots
    foot_d: float = 2.5
    arm_w: float = 2.6  # leg width (tangential)
    leg_bend_r: float = 3.0  # outer radius where the arm turns down into the foot
    leg_knee_z: float = 9.0  # the arm's top edge slopes from the collar down to this height at the foot
    root_w: float = 4.5  # leg width where it meets the collar (narrows evenly to arm_w at the foot)
    leg_inner_r: float = 0.5  # radius where the underside meets the foot (the impeller runs 0.55 inside)
    airbox_gap: float = 0.5  # the legs stay this far below the airbox that slides over the collar
    run_gap: float = 0.5  # arm underside above the impeller's backplate
    plate_t: float = 1.2
    plate_gap: float = 0.4  # hub top to plate
    collar_wall: float = 1.2
    collar_h: float = 7.0
    motor_fit: float = 0.05
    collet_slits: int = 3
    slit_phase: float = 180.0  # slits at 180/300/60 deg: clear of the legs' roots (130/230)
    slit_w: float = 0.6
    taper_l: float = 4.0  # tapered length at the collar top (the body's airbox squeezes it)
    taper: float = 0.35  # radial reduction over taper_l


F = FanParams()


def centre(p: Params = P):
    return p.board.fan_hole_x, 0.0


def heights(p: Params = P, f: FanParams = F):
    """Key z levels of the fan stack."""
    z_shroud = p.board.top_z + f.shroud_z
    z_blades = z_shroud + f.shroud_t
    z_back_tip = z_blades + f.h_tip
    z_back_eye = z_blades + f.h_eye
    z_hub_top = z_back_eye + f.back_t + f.hub_above
    z_plate = z_hub_top + f.plate_gap
    z_motor = z_plate + f.plate_t  # motor front face
    return dict(shroud=z_shroud, blades=z_blades, back_tip=z_back_tip, back_eye=z_back_eye,
                hub_top=z_hub_top, plate=z_plate, motor=z_motor, collar_top=z_motor + f.collar_h,
                shaft_end=z_motor - p.motor.shaft_l, motor_top=z_motor + p.motor.body_l + p.motor.rear_l)


def collar_r(p: Params = P, f: FanParams = F):
    return (p.motor.d + f.motor_fit) / 2 + f.collar_wall


def _revolved(points):
    """Solid of revolution about the fan axis from an (r, z) polygon."""
    return revolve(Plane.XZ * make_face(Polyline(*points, close=True)), Axis.Z)


def impeller(p: Params = P, f: FanParams = F):
    h = heights(p, f)
    r2, r1, rh = f.d2 / 2, f.eye_d / 2, f.hub_d / 2
    zs, zb, zt, ze = h["shroud"], h["blades"], h["back_tip"], h["back_eye"]
    # front shroud (flat ring with the eye), neck around the eye, conical backplate
    body = _revolved([(r1, zs), (r2, zs), (r2, zb), (r1, zb)])
    rn = f.neck_id / 2
    body += _revolved([(rn, zs - f.neck_l), (rn + f.neck_t, zs - f.neck_l), (rn + f.neck_t, zs), (rn, zs)])
    body += _revolved([(0, ze), (r1, ze), (r2, zt), (r2, zt + f.back_t), (r1, ze + f.back_t), (0, ze + f.back_t)])
    # radial blades, trimmed to the flow channel between shroud and backplate
    channel = _revolved([(rh, zb), (r2, zb), (r2, zt), (r1, ze), (rh, ze)])
    blade_len = r2 - rh
    blades = None
    for i in range(f.blades):
        blade = Pos(rh + blade_len / 2, 0, zb) * Box(blade_len, f.blade_t, ze - zb, align=MIN)
        if f.blade_angle != 90:
            blade = Pos(rh, 0, 0) * Rot(0, 0, -(90 - f.blade_angle)) * Pos(-rh, 0, 0) * blade
        blade = Rot(0, 0, 360 * i / f.blades) * blade
        blades = blade if blades is None else blades + blade
    body += blades & channel
    # nose cone turning the inflow, hub boss, shaft bore
    body += _revolved([(0, zb + 0.4), (rh * 0.6, zb + 0.4), (f.nose_d / 2, ze), (0, ze)])
    body += Pos(0, 0, ze) * Cylinder(rh, h["hub_top"] - ze, align=MIN)
    body -= Pos(0, 0, h["shaft_end"] - 0.3) * Cylinder(f.bore_d / 2, 30, align=MIN)
    x, y = centre(p)
    body = Pos(x, y, 0) * body
    body.label, body.color = "impeller", (0.95, 0.4, 0.4)
    return body


def _leg(r, p: Params = P, f: FanParams = F):
    """One leg along +x in its radial plane (r, z). It meets the collar over the collar's full height below
    the airbox and runs down outwards over the impeller, its top and underside each one straight slope,
    into a rounded bend and the foot on the board at radius r. Seen from above it narrows evenly from the
    collar to the foot."""
    from build123d import fillet
    h = heights(p, f)
    z0 = p.board.top_z
    z_arm = h["back_tip"] + f.back_t + f.run_gap  # underside at the rim: just above the backplate
    z_root = h["collar_top"] - f.taper_l - f.airbox_gap  # the airbox slides over the collar above this
    r_in, r_out = r - f.foot_d / 2, r + f.foot_d / 2
    rc = collar_r(p, f)
    pts = [(0, h["plate"]), (rc, h["plate"]), (r_in, z_arm), (r_in, z0), (r_out, z0), (r_out, f.leg_knee_z),
           (rc, z_root), (0, z_root)]
    face = make_face(Polyline(*pts, close=True))
    for (vx, vz), rad in (((r_out, f.leg_knee_z), f.leg_bend_r), ((r_in, z_arm), f.leg_inner_r)):
        try:
            face = fillet([v for v in face.vertices() if abs(v.X - vx) < 1e-6 and abs(v.Y - vz) < 1e-6], rad)
        except Exception:  # noqa: BLE001 - keep the corner sharp if the fillet fails
            pass
    side = extrude(Plane.XZ * face, f.root_w / 2, both=True)
    plan = Polyline((0, f.root_w / 2), (rc, f.root_w / 2), (r_in, f.arm_w / 2), (r_out + 1, f.arm_w / 2),
                    (r_out + 1, -f.arm_w / 2), (r_in, -f.arm_w / 2), (rc, -f.root_w / 2), (0, -f.root_w / 2), close=True)
    return side & Pos(0, 0, z0 - 1) * extrude(make_face(plan), 40)


def mount(p: Params = P, f: FanParams = F):
    h = heights(p, f)
    top = p.board.top_z
    cx, cy = centre(p)
    rc = collar_r(p, f)
    # plate and collar
    body = Pos(0, 0, h["plate"]) * Cylinder(rc, f.plate_t + f.collar_h, align=MIN)
    # two legs, each one continuous profile: a deep arm over the impeller that sweeps down into its foot
    for ang, r in f.feet:
        body += Rot(0, 0, ang) * _leg(r, p, f)
    # collet: tapered, slotted top of the collar
    ct = h["collar_top"]
    body -= Pos(0, 0, ct - f.taper_l) * (Cylinder(rc + 2, f.taper_l, align=MIN)
                                        - Cone(rc, rc - f.taper, f.taper_l, align=MIN))
    for i in range(f.collet_slits):
        body -= Rot(0, 0, f.slit_phase + 360 * i / f.collet_slits) * Pos(rc, 0, ct - f.taper_l - 1.0) * Box(
            2 * rc, f.slit_w, f.taper_l + 1.0 + 0.01, align=MIN)
    # motor bore, boss hole
    body -= Pos(0, 0, h["motor"]) * Cylinder((p.motor.d + f.motor_fit) / 2, 30, align=MIN)
    body -= Pos(0, 0, h["plate"] - 1) * Cylinder(p.motor.boss_d / 2 + 0.3, 5, align=MIN)
    # clearance around the spinning impeller: its rim and backplate, and the hub boss
    body -= Pos(0, 0, top) * Cylinder(f.d2 / 2 + 0.5, h["back_tip"] + f.back_t + f.run_gap - top, align=MIN)
    body -= Pos(0, 0, top) * Cylinder(f.hub_d / 2 + f.plate_gap, h["plate"] - top, align=MIN)
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
