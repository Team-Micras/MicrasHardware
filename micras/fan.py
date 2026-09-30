"""Suction fan: a closed radial impeller running just above the board, and its mount.

Sized for the measured motor (18k rpm no-load at 12 V, so speed-limited), following the fan study: the
impeller is as large as the layout allows (Ø26.4: nearest part 13.5 mm from the hole centre; a larger,
raised impeller would hit the encoder daughterboards and the raised drive motor), with a small eye (11),
12 radial blades and a flat 3 mm channel.

- Impeller (resin): flat front shroud 0.3 mm above the component-free Ø27 silkscreen ring (face seal),
  plus a short neck that dips into the board hole with a small radial gap. Hub pressed/glued on the shaft.
- Mount (resin): a yoke hung from the drive blocks. The motor sits in a collar, clamped by a split at its
  top and a crosswise M2 screw into a trapped nut; two arms reach back to ears on the two drive caps (an M2
  screw into a trapped nut each), and a front leg rests on the MCU. Only that foot touches the board.
"""

from dataclasses import dataclass
from math import cos, radians, sin

from build123d import Align, Axis, Box, Cylinder, Plane, Polygon, Polyline, Pos, Rot, extrude, make_face, revolve

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
    # legs (angle from +x, radius, height above the board top): one at the front, resting on the MCU (a 1.0 mm
    # tall QFN rotated 45 deg: the foot sits inside its top); the drive caps carry the rest
    feet: tuple = ((0.0, 18.0, 1.0),)
    foot_d: float = 2.5
    arm_w: float = 2.6  # leg width (tangential)
    leg_bend_r: float = 3.0  # outer radius where the arm turns down into the foot
    leg_knee_z: float = 9.0  # the arm's top edge slopes from the collar down to this height at the foot
    root_w: float = 4.5  # leg width where it meets the collar (narrows evenly to arm_w at the foot)
    leg_inner_r: float = 0.5  # radius where the underside meets the foot (the impeller runs 0.55 inside)
    leg_gap: float = 0.5  # the leg's root stays this far below the clamp
    run_gap: float = 0.5  # arm underside above the impeller's backplate
    plate_t: float = 1.2
    plate_gap: float = 0.4  # hub top to plate
    collar_wall: float = 1.2
    collar_h: float = 7.0
    motor_fit: float = 0.05
    # clamp: the collar's top is split at the front, over the front leg, and closed by an M2 screw across two
    # ears (head in one, nut trapped in the other)
    clamp_h: float = 4.0
    clamp_slit: float = 0.8
    clamp_ear_t: tuple = (1.6, 2.2)  # head side (-y), nut side (+y)
    clamp_len: float = 5.4  # radial length of the ears
    # arms to the drive caps' ears (drive.DriveParams.fan_ear)
    arm_t: float = 2.6  # arm width
    arm_z0: float = 11.0  # arm underside at the collar (it rises to the cap ear)
    tab_t: float = 1.8  # the arm's tab on the cap ear (an M2x5 then takes the whole nut)


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


def _leg(r, p: Params = P, f: FanParams = F, z_foot=0.0):
    """One leg along +x in its radial plane (r, z). It meets the collar over the collar's full height below
    the clamp and runs down outwards over the impeller, its top and underside each one straight slope,
    into a rounded bend and the foot on the board at radius r. Seen from above it narrows evenly from the
    collar to the foot."""
    from build123d import fillet
    h = heights(p, f)
    z0 = p.board.top_z + z_foot
    z_arm = h["back_tip"] + f.back_t + f.run_gap  # underside at the rim: just above the backplate
    z_root = h["collar_top"] - f.clamp_h - f.leg_gap  # the clamp's split is above this
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


def mount(p: Params = P, f: FanParams = F, d=None):
    from .drive import D, countersunk, fan_ear_top, nut_trap
    d = d or D
    h = heights(p, f)
    top = p.board.top_z
    cx, cy = centre(p)
    rc = collar_r(p, f)
    ct = h["collar_top"]
    # plate and collar, the front leg: one continuous profile, a deep arm over the impeller sweeping down into
    # its foot
    body = Pos(0, 0, h["plate"]) * Cylinder(rc, f.plate_t + f.collar_h, align=MIN)
    for ang, r, z_foot in f.feet:
        body += Rot(0, 0, ang) * _leg(r, p, f, z_foot)
    # arms to the drive caps' ears: each a web in its own vertical plane, low at the collar, rising to its tab
    zt = fan_ear_top(p, d)
    tx, ty = d.fan_ear
    for s in (1, -1):
        dx, dy = tx - cx, s * ty - cy
        ln = (dx * dx + dy * dy) ** 0.5
        x0, y0 = (rc - 0.5) * dx / ln, (rc - 0.5) * dy / ln  # (collar frame: centred on the fan)
        # the underside stays low until just short of the tab (the cap's ear is under the tab only), so the
        # arm is deep where it meets the tab
        la = ln - (rc - 0.5)
        lk = la - d.fan_ear_r - 0.3
        side = Polygon((0, f.arm_z0), (lk, f.arm_z0 + 1.0), (lk, zt), (la, zt), (la, zt + f.tab_t), (0, zt + f.tab_t),
                       align=None)
        plane = Plane(origin=(x0, y0, 0), x_dir=(dx, dy, 0), z_dir=(dy, -dx, 0))
        body += extrude(plane * side, f.arm_t / 2, both=True)
        body += Pos(dx, dy, zt) * Cylinder(d.fan_ear_r, f.tab_t, align=MIN)
        body -= Pos(dx, dy, zt + f.tab_t) * countersunk(d, depth=5, up=10)
    # clamp at the front, over the leg: a split through the collar's top and two ears with a crosswise screw
    t1, t2 = f.clamp_ear_t
    z0 = ct - f.clamp_h
    body += Pos(rc - 0.6, -f.clamp_slit / 2 - t1, z0) * Box(f.clamp_len, t1 + f.clamp_slit + t2, f.clamp_h,
                                                           align=(Align.MIN, Align.MIN, Align.MIN))
    body -= Pos(rc - 2, 0, z0) * Box(10, f.clamp_slit, 10, align=(Align.MIN, Align.CENTER, Align.MIN))
    ex, ez = rc + 2.2, ct - f.clamp_h / 2
    body -= Pos(ex, -f.clamp_slit / 2 - t1, ez) * Rot(90, 0, 0) * countersunk(d, depth=10, up=5)
    body -= Pos(ex, f.clamp_slit / 2 + t2 + 0.01, ez) * Rot(90, 0, 0) * Rot(0, 0, 30) * nut_trap(d, d.nut_t + 0.01)
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
