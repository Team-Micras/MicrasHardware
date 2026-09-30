"""Car-styled top (FDM, PETG): the top body, the front wing and the battery-box lid.

- Top body: the frame (battery box, cap posts, fan airbox, see frame.py) and a faceted wedge nose: one flat
  top plane from the lid's front edge down to the nose tip, narrowing between the diagonal sensors, with
  steep side facets and chamfered edges. The fan motor pokes up through the plane; a lug under the plane
  captures it. Windows over the front LEDs let them shine through. A keel under the tip carries the
  rounded floor skid. The body prints lying on the nose plane.
- Front wing: printed flat, screwed to the keel. It bears on the board's front edge, so crash loads go
  into the board, and it is the cheap part to replace after a crash.
- Lid: slides under the nose plane at the front, two screws at the rear; gills, shark fin and rear wing.
"""

from dataclasses import dataclass

from build123d import (Align, Axis, Box, Cylinder, Plane, Polyline, Pos, RectangleRounded, Rot, chamfer, extrude,
                       loft, make_face)

from . import drive, fan, frame
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class BodyParams:
    # wedge nose: stations (x, half width of the top, depth below the plane), rear to tip; the top plane
    # runs from the lid's front edge (plane_gap above the lid) down to (tip_x, tip_z)
    stations: tuple = ((-0.5, 16.0, 14.0), (17.5, 12.5, 10.0), (44.0, 4.2, 8.5), (57.8, 2.5, 2.4))
    tip_z: float = 6.3
    lid_overlap: float = 2.0  # the plane's rear edge overhangs the lid's front edge
    lid_gap: float = 0.15
    skin: float = 1.2  # wedge wall thickness
    facet: float = 1.2  # chamfer between the top plane and the side facets
    lug_w: float = 3.0  # the lug under the plane that holds the fan motor down
    lug_gap: float = 0.1
    led_windows: tuple = ((51.3, 2.2), (51.3, -1.4))  # (x, y) of windows over the front RGB LEDs
    led_window: float = 2.4
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
    wing_angle: float = 0.0  # flat main plane: a clean bridge between the fin and the endplates
    flap_angle: float = 45.0  # steepest angle that still prints without support
    edge_c: float = 0.6  # chamfer on the lid's top edge


B = BodyParams()


def nose_plane(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    """(x_ref, z_ref, slope): top of the wedge is z = z_ref - slope * (x - x_ref)."""
    _, x1, _, _ = frame.tray_box(p, fr)
    z_ref = frame.box_top(p, fr, d) + b.lid_t + b.lid_gap + b.skin
    return x1, z_ref, (z_ref - b.tip_z) / (b.stations[-1][0] - x1)


def plane_z(x, p: Params = P, b: BodyParams = B):
    x1, z1, k = nose_plane(p, b)
    return z1 - k * (x - x1)


def _section(x, hw, depth, p, b, inset=0.0):
    """Wedge section: vertical sides, chamfered top corners, top on the nose plane."""
    zt = plane_z(x, p, b) - inset
    hw = hw - inset
    c = min(b.facet, hw * 0.4)
    zb = zt - depth - (1.0 if inset else 0.0)  # the inner cut runs out of the bottom (open underside)
    pts = [(-hw, zb), (hw, zb), (hw, zt - c), (hw - c, zt), (-hw + c, zt), (-hw, zt - c)]
    return Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * make_face(Polyline(*pts, close=True))


def wedge(p: Params = P, b: BodyParams = B, d=drive.D):
    x1, _, _ = nose_plane(p, b)
    st = [(x1 - b.lid_overlap if i == 0 else x, hw, dep) for i, (x, hw, dep) in enumerate(b.stations)]
    body = loft([_section(x, hw, dep, p, b) for x, hw, dep in st], ruled=True)
    body -= loft([_section(x, hw, dep, p, b, inset=b.skin) for x, hw, dep in st[:-1]]
                 + [_section(st[-1][0] - 3.0, st[-1][1] + 0.3, st[-1][2], p, b, inset=b.skin)], ruled=True)
    # keel from the tip down to the rounded skid, in front of the board edge
    x0 = b.edge_x
    kz = b.skid_z + b.skid_r
    klen = b.keel_front - x0
    body += Pos(x0 + klen / 2, 0, kz) * Box(klen, b.keel_w, plane_z(b.keel_front, p, b) - kz - 0.2, align=MIN)
    body += Pos(x0 + klen / 2, 0, kz) * Rot(90, 0, 0) * Cylinder(b.skid_r, b.keel_w)
    # insert for the wing screw, facing forward
    xi = b.keel_front - b.insert_recess
    body -= Pos(b.keel_front + 1, 0, b.wing_screw_z) * Rot(0, -90, 0) * Cylinder(d.screw_clear_d / 2, 1 + b.insert_recess, align=MIN)
    body -= Pos(xi, 0, b.wing_screw_z) * Rot(0, -90, 0) * Cylinder(d.insert_d / 2, d.insert_l, align=MIN)
    body -= Pos(xi, 0, b.wing_screw_z) * Rot(0, -90, 0) * Cylinder(d.screw_clear_d / 2, d.insert_l + 1.0, align=MIN)
    # windows over the front RGB LEDs
    for lx, ly in b.led_windows:
        body -= Pos(lx, ly, 0) * Box(b.led_window, b.led_window, 40, align=MIN)
    # nothing may go behind the board edge below the board top (the edge face is the contact)
    body -= Pos(x0, 0, 0) * Box(60, 100, p.board.top_z + 0.2, align=(Align.MAX, Align.CENTER, Align.MIN))
    return body


def top_body(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    x1, z1, k = nose_plane(p, b, d, fr)
    zt = frame.box_top(p, fr, d)
    body = frame.frame(p)
    # everything in front of the box stops at the nose plane (trims the airbox tube)
    fx, fy = fan.centre(p)
    above = Pos(x1, 0, z1) * Rot(0, _deg(k), 0) * Box(200, 200, 60, align=(Align.MIN, Align.CENTER, Align.MIN))
    body -= above
    body += wedge(p, b, d)
    # room for the lid under the plane's rear overhang
    body -= Pos(x1 - b.lid_overlap - 1, 0, zt) * Box(b.lid_overlap + 1, 60, b.lid_t + b.lid_gap, align=(Align.MIN, Align.CENTER, Align.MIN))
    # airbox bore through everything (the fan motor pokes through the plane)
    h = fan.heights(p)
    r_in = p.motor.d / 2 + fr.tube_clear
    body -= Pos(fx, fy, h["plate"]) * Cylinder(r_in, 60, align=MIN)
    # one lug under the plane, behind the motor, on its rear face (between the terminal tabs)
    motor_rear = h["motor"] + p.motor.body_l + b.lug_gap
    lx = fx - r_in + b.lug_w / 2
    top = plane_z(fx - r_in, p, b) - 0.01
    lug = Pos(lx, fy, motor_rear) * Box(b.lug_w, b.lug_w, top - motor_rear, align=MIN)
    body += (lug & Pos(fx, fy, 0) * Cylinder(r_in + 0.5, 60, align=MIN)) - above
    body.label, body.color = "body", (0.85, 0.12, 0.12)
    return body


def _deg(k):
    from math import atan, degrees
    return degrees(atan(k))


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
    bosses = frame.lid_bosses(p, fr)
    r = fr.boss_d / 2
    # cover plate over the box, with tabs over the two rear screw bosses; its front edge slides under the
    # nose plane's overhang
    body = Pos((x0 + x1) / 2, 0, zt) * extrude(RectangleRounded(x1 - x0, y1 - y0, fr.corner_r), b.lid_t)
    for bx, by in bosses:
        body += Pos(bx, by, zt) * Cylinder(r, b.lid_t, align=MIN)
        body += Pos(bx + r, by, zt) * Box(2 * r, 2 * r, b.lid_t, align=MIN)
    # faceted top edge all round (faces up when printed)
    try:
        top_face = body.faces().sort_by(Axis.Z)[-1]
        body = chamfer(top_face.outer_wire().edges(), b.edge_c)
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
    for bx, by in bosses:
        body -= Pos(bx, by, zt + b.lid_t) * drive.countersunk(d, depth=5, up=10)
    # shark fin along the centre line, rising towards the rear wing
    z_top = zt + b.lid_t
    wing_x = x0 + b.wing_chord / 2 + 0.5
    xf = x1 - b.lid_overlap - 1.0  # the fin starts behind the nose plane's overhang
    fin = make_face(Polyline((xf, 0), (xf, 0.8), (wing_x + b.wing_chord / 2, b.fin_h + b.wing_z - b.fin_h + 0.5),
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
    body.label, body.color = "lid", (0.15, 0.15, 0.17)
    return body


def parts(p: Params = P):
    return {"body": top_body(p), "front_wing": front_wing(p), "lid": lid(p)}
