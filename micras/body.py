"""Car-styled top (FDM, PETG): the top body, the front wing and the battery-box lid.

- Top body: the frame (battery box, cap posts, fan airbox, see frame.py), a flat-topped fairing between
  the box and the airbox, and a slim nose with a flat underside and a 45 deg ridged top that runs down
  between the diagonal sensors to a keel whose rounded bottom is the floor skid. The box rim, fairing top
  and airbox top are one plane: the body prints upside down on it without supports.
- Front wing: printed flat, screwed to the keel. It bears on the board's front edge, so crash loads go
  into the board, and it is the cheap part to replace after a crash.
- Lid: the battery-box cover with gills, a shark fin and a rear wing, held by two screws.
"""

from dataclasses import dataclass

from build123d import (Align, Axis, Box, Cylinder, Plane, Polyline, Pos, RectangleRounded, Rot, extrude,
                       fillet, loft, make_face)

from . import drive, fan, frame
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class BodyParams:
    # nose: (x, bottom z, half width, side height) stations from the airbox to the tip; the top is a
    # 45 deg ridge (prints upside down without support), with a flat crest of 2 * crest
    nose: tuple = ((17.5, 24.0, 4.0, 3.0), (36.0, 12.0, 3.6, 2.6), (53.5, 5.4, 3.0, 1.8), (58.0, 3.0, 1.4, 0.8))
    crest: float = 0.5
    chamfer: float = 0.4  # soft lower edges of the nose
    nose_wall: float = 1.0
    nose_hollow: tuple = (26.0, 50.0)  # hollow between these x (solid in the airbox and at the tip)
    fairing_t: float = 0.9
    fairing_r: float = 2.0
    # keel / skid under the nose tip
    edge_x: float = 53.5  # board front edge
    keel_w: float = 3.6
    keel_front: float = 57.6
    skid_z: float = 0.3  # skid bottom above the floor at rest
    skid_r: float = 1.5
    wing_screw_z: float = 3.4
    insert_recess: float = 1.0  # insert starts this far behind the keel front
    # front wing
    span: float = 24.0  # stays inside the diagonal sensor caps
    plane_t: float = 0.9
    plane_z: float = 1.15  # main plane bottom (its rear edge bears on the board edge, z 1.0-2.04)
    endplate_t: float = 0.9
    endplate_top: float = 2.6  # below the sensor caps (z 2.74)
    tab: tuple = (2.0, 6.0, 5.9)  # (x thickness, y width, top z) of the wing's mounting tab
    # lid
    lid_t: float = 0.9
    lid_margin: float = 1.6
    louvre: tuple = (1.6, 3.4)  # (width, pitch) of the lid gills
    fin_t: float = 0.9
    fin_h: float = 6.0
    wing_span: float = 40.0
    wing_chord: float = 5.5
    wing_t: float = 0.9
    wing_z: float = 6.5  # above the lid top
    edge_r: float = 0.6  # soft top edge of the lid
    wing_angle: float = 0.0  # flat main plane: a clean bridge between the fin and the endplates
    flap_angle: float = 45.0  # steepest angle that still prints without support


B = BodyParams()


def _nose_section(x, zb, hw, hs, b: BodyParams, inset=0.0):
    """House-shaped section: flat bottom, short sides, 45 deg roof, small flat crest."""
    hw, hs, c, ch = hw - inset, hs, max(b.crest - inset * 0.4, 0.2), min(b.chamfer, hw / 4)
    zb = zb + inset
    top = hs + hw - c - inset * 0.4
    pts = [(-hw + ch, 0), (hw - ch, 0), (hw, ch), (hw, hs), (c, top), (-c, top), (-hw, hs), (-hw, ch)]
    face = make_face(Polyline(*pts, close=True))
    return Plane(origin=(x, 0, zb), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * face


def nose(p: Params = P, b: BodyParams = B, d=drive.D):
    """Slim nose from the airbox down to the keel and skid (part of the top body)."""
    top = p.board.top_z
    body = loft([_nose_section(*st, b) for st in b.nose])
    # hollow in the middle (lighter); solid in the airbox and at the tip for the insert
    xa, xb = b.nose_hollow

    def station(x):
        for (x0, *a0), (x1, *a1) in zip(b.nose, b.nose[1:]):
            if x0 <= x <= x1:
                t = (x - x0) / (x1 - x0)
                return (x, *[u + t * (v - u) for u, v in zip(a0, a1)])
    inner = [st for st in b.nose if xa < st[0] < xb]
    body -= loft([_nose_section(*s, b, inset=b.nose_wall) for s in (station(xa), *inner, station(xb))])
    # keel from the nose tip down to the rounded skid, in front of the board edge
    x0 = b.edge_x
    kz = b.skid_z + b.skid_r
    klen = b.keel_front - x0
    body += Pos(x0 + klen / 2, 0, kz) * Box(klen, b.keel_w, 6.0 - kz + 0.5, align=MIN)
    body += Pos(x0 + klen / 2, 0, kz) * Rot(90, 0, 0) * Cylinder(b.skid_r, b.keel_w)
    # insert for the wing screw, facing forward
    xi = b.keel_front - b.insert_recess
    body -= Pos(b.keel_front + 1, 0, b.wing_screw_z) * Rot(0, -90, 0) * Cylinder(d.screw_clear_d / 2, 1 + b.insert_recess, align=MIN)
    body -= Pos(xi, 0, b.wing_screw_z) * Rot(0, -90, 0) * Cylinder(d.insert_d / 2, d.insert_l, align=MIN)
    body -= Pos(xi, 0, b.wing_screw_z) * Rot(0, -90, 0) * Cylinder(d.screw_clear_d / 2, d.insert_l + 1.0, align=MIN)
    # nothing may go behind the board edge below the board top (the edge face is the contact)
    body -= Pos(x0, 0, 0) * Box(40, 100, top + 0.2, align=(Align.MAX, Align.CENTER, Align.MIN))
    return body


def fairing(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    """Flat-topped hollow fairing joining the battery box to the airbox (top flush with the box rim)."""
    _, x1, _, _ = frame.tray_box(p, fr)
    zt = frame.box_top(p, fr, d)
    z0 = d.tray_bottom_z
    fx, _ = fan.centre(p)
    r_out = p.motor.d / 2 + fr.tube_clear + fr.tube_wall
    xa, xe = x1 - 1.0, fx
    outer = Pos((xa + xe) / 2, 0, z0) * extrude(RectangleRounded(xe - xa, 2 * r_out, b.fairing_r), zt - z0)
    inner = Pos((xa + xe) / 2, 0, z0 + b.fairing_t) * extrude(
        RectangleRounded(xe - xa - 2 * b.fairing_t, 2 * r_out - 2 * b.fairing_t, b.fairing_r - b.fairing_t / 2),
        zt - z0 - 2 * b.fairing_t)
    return outer - inner


def top_body(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    body = frame.frame(p) + nose(p, b, d) + fairing(p, b, d, fr)
    zt = frame.box_top(p, fr, d)
    # re-cut the front lid screw (its boss sits inside the fairing)
    (fbx, fby), _ = frame.lid_bosses(p, fr)
    body -= Pos(fbx, fby, zt) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
    body -= Pos(fbx, fby, zt) * Cylinder(d.screw_clear_d / 2, d.screw_l - 0.9 + 0.5, align=MAX)
    # airbox bore through everything, then the motor lugs again
    h = fan.heights(p)
    fx, fy = fan.centre(p)
    r_in = p.motor.d / 2 + fr.tube_clear
    body -= Pos(fx, fy, h["plate"]) * Cylinder(r_in, zt - h["plate"] + 1, align=MIN)
    body += frame.lugs(p, fr)
    body.label, body.color = "body", (0.85, 0.12, 0.12)
    return body


def front_wing(p: Params = P, b: BodyParams = B, d=drive.D):
    """Printed flat (main plane on the bed); screwed to the keel through its central tab."""
    x0 = b.edge_x
    chord = b.keel_front - x0 + 0.4
    body = Pos(x0 + chord / 2, 0, b.plane_z) * Box(chord, b.span, b.plane_t, align=MIN)
    body -= Pos(x0 + chord / 2 - 0.2, 0, b.plane_z) * Box(chord, b.keel_w + 0.3, b.plane_t, align=MIN)  # keel slot
    for sy in (1, -1):
        body += Pos(x0 + chord / 2 + 0.3, sy * (b.span / 2 - b.endplate_t / 2), b.plane_z) * Box(
            chord + 0.6, b.endplate_t, b.endplate_top - b.plane_z, align=MIN)
    tx, tw, tz = b.tab
    body += Pos(b.keel_front + 0.1 + tx / 2, 0, b.plane_z) * Box(tx, tw, tz - b.plane_z, align=MIN)
    # tie the two halves across the slot, in front of the keel
    body += Pos(b.keel_front + 0.1 + tx / 2, 0, b.plane_z) * Box(tx, b.span - 1, b.plane_t, align=MIN)
    seat = b.keel_front - b.insert_recess - d.insert_l + d.screw_l
    body -= Pos(seat, 0, b.wing_screw_z) * Rot(0, 90, 0) * drive.countersunk(d, depth=d.screw_l, up=10)
    body.label, body.color = "front_wing", (0.15, 0.15, 0.17)
    return body


def lid(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    x0, x1, y0, y1 = frame.tray_box(p, fr)
    zt = frame.box_top(p, fr, d)
    (fbx, _), (rbx, _) = frame.lid_bosses(p, fr)
    r = fr.boss_d / 2
    # cover plate over the box, with tabs over the two screw bosses
    body = Pos((x0 + x1) / 2, 0, zt) * extrude(RectangleRounded(x1 - x0, y1 - y0, fr.corner_r), b.lid_t)
    for bx in (fbx, rbx):
        body += Pos(bx, 0, zt) * Cylinder(r, b.lid_t, align=MIN)
        body += Pos((bx + (x0 + x1) / 2) / 2, 0, zt) * Box(abs(bx - (x0 + x1) / 2), 2 * r, b.lid_t, align=MIN)
    # soft top edge all round (faces up when printed)
    try:
        top_face = body.faces().sort_by(Axis.Z)[-1]
        body = fillet(top_face.outer_wire().edges(), b.edge_r)
    except Exception:  # noqa: BLE001 - cosmetic only
        pass
    # longitudinal gills, like an engine cover's louvres
    m = b.lid_margin
    gw, pitch = b.louvre
    n = int((y1 - y0 - 2 * m) // pitch)
    for j in range(n):
        yc = (j - (n - 1) / 2) * pitch
        if abs(yc) < b.fin_t / 2 + gw:  # solid strip under the fin
            continue
        body -= Pos((x0 + x1) / 2 - 1.5, yc, zt) * Box(x1 - x0 - 2 * m - 5, gw, b.lid_t, align=MIN)
    for bx in (fbx, rbx):
        body -= Pos(bx, 0, zt + b.lid_t) * drive.countersunk(d, depth=5, up=10)
    # shark fin along the centre line, rising towards the rear wing
    z_top = zt + b.lid_t
    wing_x = x0 + b.wing_chord / 2 + 0.5
    fin = make_face(Polyline((x1 - 1, 0), (x1 - 1, 0.8), (wing_x + b.wing_chord / 2, b.fin_h + b.wing_z - b.fin_h + 0.5),
                             (wing_x - b.wing_chord / 2, b.wing_z + 0.5), (wing_x - b.wing_chord / 2, 0), close=True))
    body += Pos(0, b.fin_t / 2, z_top) * (Plane.XZ * extrude(fin, b.fin_t))
    # rear wing on the fin and two endplates
    wing = Rot(0, b.wing_angle, 0) * Box(b.wing_chord, b.wing_span, b.wing_t)
    body += Pos(wing_x, 0, z_top + b.wing_z) * wing
    flap = Rot(0, b.flap_angle, 0) * Box(b.wing_chord * 0.55, b.wing_span, b.wing_t * 0.9)
    body += Pos(wing_x - b.wing_chord * 0.55, 0, z_top + b.wing_z + 1.6) * flap
    for sy in (1, -1):
        plate = make_face(Polyline((wing_x + b.wing_chord / 2 + 1.0, 0), (wing_x + b.wing_chord / 2 + 1.0, b.wing_z + 1.0),
                                   (wing_x - b.wing_chord * 0.3, b.wing_z + 3.8), (wing_x - b.wing_chord - 0.6, b.wing_z + 3.8),
                                   (wing_x - b.wing_chord - 0.6, 0), close=True))
        body += Pos(0, sy * (b.wing_span / 2 - b.wing_t / 2) + b.wing_t / 2, z_top) * (Plane.XZ * extrude(plate, b.wing_t))
    body -= Pos(fbx, 0, zt + b.lid_t) * Cylinder(d.csk_d / 2 + 0.05, 20, align=MIN)
    body.label, body.color = "lid", (0.15, 0.15, 0.17)
    return body


def parts(p: Params = P):
    return {"body": top_body(p), "front_wing": front_wing(p), "lid": lid(p)}
