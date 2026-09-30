"""Car-styled top (FDM, PETG): the one-piece top body and the battery-box lid.

- Top body: the frame (battery box, cap posts, fan airbox tube, see frame.py) plus a smooth hollow nose
  cone that runs from the airbox forward and down between the diagonal sensors to the front wing. The
  wing bears on the board's front edge, so crash loads go into the board, and the keel under the nose
  tip is the floor skid.
- Lid: the battery-box cover with gills, a shark fin and a rear wing, held by two screws.
"""

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Ellipse, Plane, Polyline, Pos, Rot, extrude, loft, make_face

from . import drive, fan, frame
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class BodyParams:
    # nose cone: (x, centre z above the board top, half width, half height) sections, rear to tip
    nose: tuple = ((17.5, 28.0, 3.9, 3.6), (38.0, 14.0, 3.8, 3.0), (53.5, 4.6, 3.3, 2.7), (58.5, 1.3, 1.3, 0.9))
    nose_hollow_from: float = 26.0  # the nose is solid inside/near the airbox tube, hollow from here on
    fairing_t: float = 0.9  # engine-cover fairing shell between the battery box and the airbox
    nose_wall: float = 0.9
    # front wing
    edge_x: float = 53.5  # board front edge
    span: float = 24.0  # stays inside the diagonal sensor caps
    chord: float = 4.5
    plane_t: float = 0.9
    plane_z: float = 1.15  # main plane bottom (its rear edge bears on the board edge, z 1.0-2.04)
    endplate_t: float = 0.9
    endplate_h: float = 1.65  # stays below the sensor caps (z 2.74)
    keel_w: float = 3.6
    keel_len: float = 3.5  # in front of the board edge
    skid_z: float = 0.3  # skid bottom above the floor at rest
    skid_r: float = 1.5
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
    wing_angle: float = 12.0


B = BodyParams()


def nose_wing(p: Params = P, b: BodyParams = B):
    """Hollow nose cone from the airbox tube to the front wing, with the keel/skid and the wing."""
    top = p.board.top_z
    x0 = b.edge_x
    outer = [Plane(origin=(x, 0, top + z), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * Ellipse(hw, hh) for x, z, hw, hh in b.nose]
    # hollow, but solid where it merges into the airbox tube (the bore is cut later) and at the tip
    (xa, za, wa, ha), (xb, zb, wb, hb) = b.nose[0], b.nose[1]
    t = (b.nose_hollow_from - xa) / (xb - xa)
    first = (b.nose_hollow_from, za + t * (zb - za), wa + t * (wb - wa), ha + t * (hb - ha))
    inner = [Plane(origin=(x, 0, top + z), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * Ellipse(hw - b.nose_wall, hh - b.nose_wall)
             for x, z, hw, hh in (first, *b.nose[1:-1])]
    body = loft(outer) - loft(inner)
    # main plane: rear edge on the board's front edge; endplates below the sensor caps
    body += Pos(x0 + b.chord / 2, 0, b.plane_z) * Box(b.chord, b.span, b.plane_t, align=MIN)
    for sy in (1, -1):
        body += Pos(x0 + b.chord / 2 + 0.6, sy * (b.span / 2 - b.endplate_t / 2), b.plane_z - 0.4) * Box(
            b.chord + 1.2, b.endplate_t, b.endplate_h, align=MIN)
    # keel from the nose tip down to the rounded skid
    kz = b.skid_z + b.skid_r
    body += Pos(x0 + b.keel_len / 2, 0, kz) * Box(b.keel_len, b.keel_w, top + 3.0 - kz, align=MIN)
    body += Pos(x0 + b.keel_len / 2, 0, kz) * Rot(90, 0, 0) * Cylinder(b.skid_r, b.keel_w)
    # nothing may go behind the board edge below the board top (the edge face is the contact)
    body -= Pos(x0, 0, 0) * Box(40, 100, top + 0.2, align=(Align.MAX, Align.CENTER, Align.MIN))
    return body


def fairing(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    """Hollow wedge from the battery box's front edge sloping down to the airbox tube (side profile)."""
    _, x1, _, _ = frame.tray_box(p, fr)
    zt = frame.box_top(p, fr, d)
    z0 = d.tray_bottom_z
    fx, _ = fan.centre(p)
    h = fan.heights(p)
    tube_top = h["motor"] + p.motor.body_l + fr.lug_gap
    r_out = p.motor.d / 2 + fr.tube_clear + fr.tube_wall
    xe = fx - 1.5

    def wedge(inset):
        pts = [(x1 - 0.5 + inset, z0 + inset), (xe, z0 + inset), (xe, tube_top - inset), (x1 - 0.5 + inset, zt - inset)]
        return Pos(0, r_out - inset, 0) * (Plane.XZ * extrude(make_face(Polyline(*pts, close=True)), 2 * (r_out - inset)))

    return wedge(0) - wedge(b.fairing_t)


def top_body(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    body = frame.frame(p) + nose_wing(p, b) + fairing(p, b)
    # re-cut the front lid screw (its boss sits inside the fairing)
    (fbx, fby), _ = frame.lid_bosses(p, fr)
    zt = frame.box_top(p, fr, d)
    body -= Pos(fbx, fby, zt) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
    body -= Pos(fbx, fby, zt) * Cylinder(d.screw_clear_d / 2, d.screw_l - 0.9 + 0.5, align=MAX)
    # keep the airbox bore clear where the nose joins the tube
    h = fan.heights(p)
    fx, fy = fan.centre(p)
    r_in = p.motor.d / 2 + frame.FR.tube_clear
    body -= Pos(fx, fy, h["plate"]) * Cylinder(r_in, h["motor"] + p.motor.body_l - h["plate"], align=MIN)
    body.label, body.color = "body", (0.85, 0.12, 0.12)
    return body


def lid(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    x0, x1, y0, y1 = frame.tray_box(p, fr)
    zt = frame.box_top(p, fr, d)
    (fbx, _), (rbx, _) = frame.lid_bosses(p, fr)
    r = fr.boss_d / 2
    # cover plate over the box, with tabs over the two screw bosses
    body = Pos((x0 + x1) / 2, 0, zt) * Box(x1 - x0, y1 - y0, b.lid_t, align=MIN)
    for bx in (fbx, rbx):
        body += Pos(bx, 0, zt) * Cylinder(r, b.lid_t, align=MIN)
        body += Pos((bx + (x0 + x1) / 2) / 2, 0, zt) * Box(abs(bx - (x0 + x1) / 2), 2 * r, b.lid_t, align=MIN)
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
    flap = Rot(0, b.wing_angle + 18, 0) * Box(b.wing_chord * 0.55, b.wing_span, b.wing_t * 0.9)
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
    return {"body": top_body(p), "lid": lid(p)}
