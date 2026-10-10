"""Suction fan: a closed radial impeller running just above the board, and its mount.

Sized for the fan motor (Ø9.97 x 23.08, 54k rpm at 7.6 V: about 87k rpm with no load at 12.3 V, so it is limited
by its power and heat, not its speed) following docs/fan_study.md: a Ø22 impeller (eye 10, twelve straight radial
blades from hub to tip, the channel tapering from 3 at the eye to 2 at the tip; the CFD found the curved-inlet and
backward-curved variants no better) run at about 6.3 V average (PWM) for about 1000 Pa, 4 N with
the skirt; never at full voltage (79k rpm, 17 W). Its hub is glued onto the motor's 9T pinion (it doesn't come
off), gripping 3.8 of its 4.6 mm.

- Impeller (resin): flat front shroud 0.3 mm above the component-free Ø27 silkscreen ring (face seal),
  plus a short neck that dips into the board hole with a small radial gap. The hub's bore is the pinion's own
  9-tooth outline with a small clearance: it slides on, the teeth key it, thin CA wicked in holds it; balance it
  after.
- Mount (resin): a yoke hung from the drive blocks. The motor sits in a collar, clamped by a split at its
  top and a crosswise M2 screw into a trapped nut; two arms reach back to ears on the two drive caps (an M2
  screw into a trapped nut each). It doesn't touch the board (a front leg on the MCU was dropped: the two
  arms hold it).
"""

from dataclasses import dataclass
from math import cos, radians, sin

from build123d import Align, Axis, Box, Cylinder, Plane, Polygon, Polyline, Pos, Rot, extrude, make_face, mirror, revolve

from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FanParams:
    # impeller
    d2: float = 22.0  # docs/fan_study.md (the layout fits up to 26.4: nearest component 13.5 from the centre)
    eye_d: float = 10.0
    # blades: radial at the outlet (near shut-off the outlet angle hardly matters, and radial blades don't bend
    # spinning), but curved at the inlet to meet the air, which reaches the eye at about 24 deg from the tangent
    # (straight blades meeting it edge-on lost about 15 % of the shaft power, docs/fan_study.md). Twelve curved
    # inlets would block the eye, so six main blades carry them, with six short radial splitters between them.
    # three blade styles to compare (all Ø22, same channel, hub and seal; print all three, docs/fan_study.md):
    #   "radial"    12 straight radial blades, hub to tip (the first design)
    #   "inducer"   6 main blades with curved inlets, radial from blade_knee_r out, and 6 radial splitters
    #   "backward"  7 backward-curved blades (a vacuum-cleaner fan): beta1 at the inlet easing to beta2_backward
    style: str = "radial"  # the default: the CFD found no gain from the curved inlets (docs/fan_study.md, CFD results)
    blades: int = 6  # main blades ("inducer")
    radial_blades: int = 12
    backward_blades: int = 7
    beta2_backward: float = 35.0  # outlet angle from the tangent
    blade_t: float = 0.55
    blade_le_r: float = 4.5  # the main blades' leading edge, inside the eye
    beta1: float = 26.0  # blade angle from the tangent at the leading edge
    blade_knee_r: float = 8.0  # radial from here to the tip (the angle eases from beta1 to 90 deg)
    splitter_r: float = 7.0  # the splitters' inner end (straight, radial)
    spin: int = 1  # +1: counter-clockwise seen from above (the motor's side); the inlets lead into the turn
    h_eye: float = 3.0  # blade height at the eye
    h_tip: float = 2.0  # blade height at the tip (the channel tapers: conical backplate)
    shroud_t: float = 0.6  # front shroud (bottom)
    back_t: float = 0.8  # backplate (top)
    shroud_z: float = 0.3  # front shroud underside above the board top (face seal on the Ø27 ring)
    neck_id: float = 13.4  # neck under the shroud: OD 14.4 in the Ø15 board hole
    neck_t: float = 0.5
    neck_l: float = 1.2  # dips 0.9 mm into the board hole
    hub_d: float = 6.0  # round the pinion's Ø3.3 tips, at least 1 mm of wall
    hub_motor_gap: float = 0.3  # hub top below the motor's front boss (the hub rises through the plate's hole)
    nose_d: float = 6.0  # flow-turning cone under the hub
    vent_d: float = 0.8  # through the nose, from the bore's bottom
    # bore: the pinion's outline (9T M0.3, standard involute: tips Ø3.30, roots Ø1.98) grown by pinion_fit all
    # round; its ridges stop at pinion_ridge_d, clear of the pinion's roots, and its mouth flares for the lead-in
    pinion_fit: float = 0.06
    pinion_ridge_d: float = 2.3
    pinion_lead: float = 0.3
    # mount
    # legs down to the board (angle from +x, radius, height above the board top), e.g. (0.0, 18.0, 1.0) on the
    # MCU at the front (a 1.0 mm tall QFN): none, the arms to the drive caps carry the mount
    feet: tuple = ()
    foot_d: float = 2.5
    arm_w: float = 2.6  # leg width (tangential)
    leg_bend_r: float = 3.0  # outer radius where the arm turns down into the foot
    leg_knee_z: float = 9.2  # the arm's top edge slopes from the collar down to this height at the foot
    root_w: float = 4.5  # leg width where it meets the collar (narrows evenly to arm_w at the foot)
    leg_inner_r: float = 0.5  # radius where the underside meets the foot (the impeller runs 0.55 inside)
    leg_gap: float = 0.5  # the leg's root stays this far below the clamp
    run_gap: float = 0.5  # arm underside above the impeller's backplate
    plate_t: float = 1.2
    plate_above: float = 1.9  # plate underside above the backplate at the eye
    plate_gap: float = 0.4  # radial, round the hub in the plate's hole
    collar_wall: float = 1.2
    collar_h: float = 7.0
    motor_fit: float = 0.10  # slides in, the clamp holds it (0.15 at 2.5 s was a little loose)
    # clamp: the collar's top is split at the front, over the front leg, and closed by an M2 screw across two
    # ears (head in one, nut trapped in the other)
    clamp_h: float = 6.0  # holds the whole nut, flats up and down, with 0.9 mm walls (at 4.0 the trap broke out of
    # the ears' top and bottom, and the nut turned with the screw)
    clamp_slit: float = 0.8
    clamp_ear_t: tuple = (1.6, 2.2)  # head side (-y), nut side (+y)
    clamp_nut_wall: float = 0.6  # resin between the nut trap's nearest corner and the motor bore: the screw as close
    # to the motor as that allows (it was 0.5 mm further out)
    clamp_end: float = 2.6  # the ears run this far past the screw's axis (0.2 mm past the trap's corner)
    # arms to the drive caps' ears (drive.DriveParams.fan_ear)
    arm_t: float = 2.6  # arm width
    arm_z0: float = 11.2  # arm underside at the collar (it rises to the cap ear)
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
    z_plate = z_back_eye + f.back_t + f.plate_above
    z_motor = z_plate + f.plate_t  # motor front face
    z_hub_top = z_motor - p.fan_motor.boss_l - f.hub_motor_gap
    return dict(shroud=z_shroud, blades=z_blades, back_tip=z_back_tip, back_eye=z_back_eye,
                hub_top=z_hub_top, plate=z_plate, motor=z_motor, collar_top=z_motor + f.collar_h,
                shaft_end=z_motor - p.fan_motor.shaft_l, motor_top=z_motor + p.fan_motor.body_l + p.fan_motor.rear_l)


def collar_r(p: Params = P, f: FanParams = F):
    return (p.fan_motor.d + f.motor_fit) / 2 + f.collar_wall


def clamp_screw(p: Params = P, f: FanParams = F, d=None):
    """(x, z) of the collar clamp's crosswise screw, in the fan's frame (x forward from its axis)."""
    from .drive import D
    d = d or D
    corner = d.nut_af / 3 ** 0.5  # the trap's flats are up and down: a corner points at the motor
    y_in = f.clamp_slit / 2 + f.clamp_ear_t[1] - d.nut_t  # the nut's inner face, off the slit's middle
    rw = (p.fan_motor.d + f.motor_fit) / 2 + f.clamp_nut_wall
    return corner + (rw * rw - y_in * y_in) ** 0.5, heights(p, f)["collar_top"] - f.clamp_h / 2


def _revolved(points):
    """Solid of revolution about the fan axis from an (r, z) polygon."""
    return revolve(Plane.XZ * make_face(Polyline(*points, close=True)), Axis.Z)


def impeller(p: Params = P, f: FanParams = F, shaft=None):
    """The impeller on the fan motor's 9T pinion, or with shaft = (diameter, bore, length out of the motor's front
    face) on a plain round shaft instead (trials.py): a blind bore whose bottom stops the shaft where the pinion
    would put the hub."""
    h = heights(p, f)
    r2, r1, rh = f.d2 / 2, f.eye_d / 2, f.hub_d / 2
    zs, zb, zt, ze = h["shroud"], h["blades"], h["back_tip"], h["back_eye"]
    # front shroud (flat ring with the eye), neck around the eye, conical backplate
    body = _revolved([(r1, zs), (r2, zs), (r2, zb), (r1, zb)])
    rn = f.neck_id / 2
    body += _revolved([(rn, zs - f.neck_l), (rn + f.neck_t, zs - f.neck_l), (rn + f.neck_t, zs), (rn, zs)])
    body += _revolved([(0, ze), (r1, ze), (r2, zt), (r2, zt + f.back_t), (r1, ze + f.back_t), (0, ze + f.back_t)])
    # blades (main ones with curved inlets, radial splitters between them), trimmed to the flow channel between
    # shroud and backplate
    channel = _revolved([(rh, zb), (r2, zb), (r2, zt), (r1, ze), (rh, ze)])
    def straight(r0):
        return Pos(r0, -f.blade_t / 2, zb) * Box(r2 + 0.5 - r0, f.blade_t, ze - zb, align=(Align.MIN, Align.MIN, Align.MIN))

    if f.style == "radial":
        kinds = [(straight(rh), f.radial_blades, 0.0)]
    elif f.style == "inducer":
        main = Pos(0, 0, zb) * extrude(_blade_plan(f, r2, 90.0, f.blade_knee_r), ze - zb)
        kinds = [(main, f.blades, 0.0), (straight(f.splitter_r), f.blades, 180 / f.blades)]
    elif f.style == "backward":
        main = Pos(0, 0, zb) * extrude(_blade_plan(f, r2, f.beta2_backward, r2 + 0.5), ze - zb)
        kinds = [(main, f.backward_blades, 0.0)]
    else:
        raise ValueError(f"FanParams.style {f.style!r}")
    blades = None
    for blade, n, a0 in kinds:
        for i in range(n):
            b = Rot(0, 0, a0 + 360 * i / n) * blade
            blades = b if blades is None else blades + b
    body += blades & channel
    # the spin's direction, engraved 0.3 deep in the backplate's top: an arc with its head pointing the way it turns
    r_a, w_a, z_back = 8.0, 0.8, lambda r: ze + f.back_t + (zt - ze) * (r - r1) / (r2 - r1)
    arc = [(r * cos(radians(a)), r * sin(radians(a))) for r in (r_a - w_a / 2,) for a in range(-20, 21, 4)]
    arc += [(r * cos(radians(a)), r * sin(radians(a))) for r in (r_a + w_a / 2,) for a in range(20, -21, -4)]
    head = [(r * cos(radians(a)), r * sin(radians(a))) for r, a in ((r_a - 1.0, 20), (r_a, 32), (r_a + 1.0, 20))]
    arrow = extrude(make_face(Polyline(*arc, close=True)), 20) + extrude(make_face(Polyline(*head, close=True)), 20)
    if f.spin < 0:
        arrow = mirror(arrow, Plane.XZ)
    layer = _revolved([(r_a - 1.2, z_back(r_a - 1.2) - 0.3), (r_a + 1.2, z_back(r_a + 1.2) - 0.3), (r_a + 1.2, 20), (r_a - 1.2, 20)])
    if f.style != "radial":  # (straight radial blades work either way round)
        body -= Pos(0, 0, -5) * arrow & layer
    # nose cone turning the inflow, hub boss, shaft bore
    body += _revolved([(0, zb + 0.4), (rh * 0.6, zb + 0.4), (f.nose_d / 2, ze), (0, ze)])
    body += Pos(0, 0, ze) * Cylinder(rh, h["hub_top"] - ze, align=MIN)
    if shaft is None:
        z_end = h["shaft_end"] - 0.3
        body -= Pos(0, 0, z_end) * pinion_bore(p, f, h["hub_top"] - z_end)
    else:
        from build123d import Cone
        _, bore, length = shaft
        z_end = h["motor"] - length  # the shaft's end bottoms here
        if z_end - (zb + 0.4) < 0.4:
            raise ValueError(f"a {length} mm shaft leaves {z_end - zb - 0.4:.2f} mm under the bore (0.4 at least)")
        body -= Pos(0, 0, z_end) * Cylinder(bore / 2, h["hub_top"] - z_end + 0.01, align=MIN)
        body -= Pos(0, 0, h["hub_top"] - f.pinion_lead) * Cone(bore / 2, bore / 2 + f.pinion_lead, f.pinion_lead + 0.01, align=MIN)
    # vent from the blind bore's bottom out through the nose: printed hub-side down, the bore would be a suction
    # cup (the shaft and its glue close it once fitted)
    body -= Pos(0, 0, zb - 1) * Cylinder(f.vent_d / 2, z_end - zb + 1.01, align=MIN)
    x, y = centre(p)
    body = Pos(x, y, 0) * body
    body.label, body.color = "impeller", (0.95, 0.4, 0.4)
    return body


def pinion_bore(p: Params = P, f: FanParams = F, depth=5.0):
    """The hub's bore from z = 0 up to depth: the fan motor's 9T pinion grown by pinion_fit, with a lead-in at
    the top."""
    import py_gearworks as pg
    from build123d import Cone, offset
    gear = pg.SpurGear(number_of_teeth=9, module=p.fan_motor.shaft_d / 11, height=1.0, enable_undercut=True)
    outline = gear.build_part().faces().sort_by(Axis.Z)[0]
    bore = extrude(offset(outline, f.pinion_fit), depth, dir=(0, 0, 1)) + Cylinder(f.pinion_ridge_d / 2, depth, align=MIN)
    r_tip = p.fan_motor.shaft_d / 2 + f.pinion_fit
    lead = Pos(0, 0, depth - f.pinion_lead) * Cone(r_tip - f.pinion_lead, r_tip + f.pinion_lead, f.pinion_lead + 0.01,
                                                   align=MIN)
    return bore + lead


def impeller_variants(p: Params = P, f: FanParams = F):
    """The three blade styles to print and compare: impeller_radial, impeller_inducer, impeller_backward."""
    from dataclasses import replace
    out = {}
    for style in ("radial", "inducer", "backward"):
        part = impeller(p, replace(f, style=style))
        part.label = f"impeller_{style}"
        out[part.label] = part
    return out


def _blade_plan(f: FanParams, r2, beta2, rk):
    """A curved blade's outline seen from above: its camber line runs from the leading edge (blade_le_r, at beta1
    from the tangent) out to rk, the angle easing to beta2, then (if rk is inside the tip) straight and radial to
    past the tip, ending on +x. Going outwards the line turns against the spin, so the inlet leans into the turn
    and scoops the air in (and a backward-curved blade trails behind)."""
    import numpy as np
    from build123d import Polyline
    r0, b1, b2 = f.blade_le_r, radians(f.beta1), radians(beta2)
    rs = np.linspace(r0, rk, 60)
    beta = b1 + (b2 - b1) * (1 - np.cos(np.pi * (rs - r0) / (rk - r0))) / 2
    # theta at rk is 0; inwards it grows in the spin's sense: d(theta)/dr = -spin / (r tan beta)
    g = 1 / (rs * np.tan(beta))
    theta = f.spin * np.concatenate([np.cumsum(((g[1:] + g[:-1]) / 2 * np.diff(rs))[::-1])[::-1], [0.0]])
    pts = [(r * np.cos(t), r * np.sin(t)) for r, t in zip(rs, theta)] + ([(r2 + 0.5, 0.0)] if rk < r2 else [])
    pts = np.array(pts)
    # the outline: the camber line offset by half the thickness each side
    d = np.gradient(pts, axis=0)
    n = np.column_stack([-d[:, 1], d[:, 0]]) / np.linalg.norm(d, axis=1)[:, None] * f.blade_t / 2
    side_a, side_b = pts + n, pts - n
    return make_face(Polyline(*[tuple(q) for q in side_a], *[tuple(q) for q in side_b[::-1]], close=True))


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
        # the tab sits in the ear's recess (it locates the mount before the screws go in)
        zr = zt - d.fan_recess
        body += Pos(dx, dy, zr) * Cylinder(d.fan_tab_r, zt + f.tab_t - zr, align=MIN)
        body -= Pos(dx, dy, zr + f.tab_t) * countersunk(d, depth=5, up=10)
    # clamp at the front, over the leg: a split through the collar's top and two ears with a crosswise screw
    t1, t2 = f.clamp_ear_t
    z0 = ct - f.clamp_h
    ex, ez = clamp_screw(p, f, d)
    body += Pos(rc - 0.6, -f.clamp_slit / 2 - t1, z0) * Box(ex + f.clamp_end - (rc - 0.6), t1 + f.clamp_slit + t2,
                                                           f.clamp_h, align=(Align.MIN, Align.MIN, Align.MIN))
    body -= Pos(rc - 2, 0, z0) * Box(10, f.clamp_slit, 10, align=(Align.MIN, Align.CENTER, Align.MIN))
    body -= Pos(ex, -f.clamp_slit / 2 - t1, ez) * Rot(90, 0, 0) * countersunk(d, depth=10, up=5)
    body -= Pos(ex, f.clamp_slit / 2 + t2 + 0.01, ez) * Rot(90, 0, 0) * nut_trap(d, d.nut_t + 0.01)  # flats up and down
    # motor bore, boss hole
    body -= Pos(0, 0, h["motor"]) * Cylinder((p.fan_motor.d + f.motor_fit) / 2, 30, align=MIN)
    # clearance around the spinning impeller: its rim and backplate, and the hub boss
    body -= Pos(0, 0, top) * Cylinder(f.d2 / 2 + 0.5, h["back_tip"] + f.back_t + f.run_gap - top, align=MIN)
    body -= Pos(0, 0, top) * Cylinder(f.hub_d / 2 + f.plate_gap, h["motor"] - top, align=MIN)  # through the plate
    body = Pos(cx, cy, 0) * body
    body.label, body.color = "fan_mount", (0.9, 0.55, 0.2)
    return body


def fan_motor(p: Params = P, f: FanParams = F):
    from .purchased import motor
    h = heights(p, f)
    cx, cy = centre(p)
    m = Pos(cx, cy, h["motor"]) * Rot(180, 0, 0) * motor(p.fan_motor, "fan_motor")  # shaft down
    m.label = "fan_motor"
    return m


def parts(p: Params = P, f: FanParams = F):
    return {"impeller": impeller(p, f), "fan_mount": mount(p, f), "fan_motor": fan_motor(p, f)}
