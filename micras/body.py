"""Car-styled top (FDM, PETG): the top body and the battery-box lid.

- Top body: the frame (battery box, cap posts, fan airbox, see frame.py) and a short faceted cowl: one flat
  top plane that starts flush with the rim and the lid's top and runs down over the fan motor, ending just
  in front of it, with steep side facets and chamfered edges. The fan motor pokes up through the plane
  (the airbox clamps it through the fan mount's collet). The front bumper is a separate part (front.py).
- Lid: flush in the rim, screwed at its four corner ears; gills, shark fin and rear wing.
"""

from dataclasses import dataclass

from build123d import (Align, Axis, Box, Cylinder, Plane, Polyline, Pos, RectangleRounded, Rot, chamfer, extrude,
                       loft, make_face, offset)

from . import drive, fan, frame
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class BodyParams:
    # cowl: stations (x, half width of the top, depth below the plane), rear to front; the top plane runs
    # from the box front (flush with the rim) down at `slope`; the last station is the cut face just in
    # front of the fan airbox
    stations: tuple = ((0.0, 16.0, 14.0), (17.5, 12.5, 10.0), (27.0, 9.7, 9.5))
    slope: float = 0.647  # 33 deg: the body prints lying on this plane
    skin: float = 1.2  # cowl wall thickness
    facet: float = 1.2  # chamfer between the top plane and the side facets
    front_c: float = 1.5  # chamfer on the cut face's top edges
    lip_d: float = 1.5  # nose lip over the lid's front edge
    lip_hw: float = 14.5  # its half width (inside the cowl's facets)
    lip_t: float = 0.45
    lip_gap: float = 0.05  # lid tongue under the lip (0.4 thick)
    # lid
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
    edge_c: float = 0.3  # chamfer on the lid's top edge


B = BodyParams()


def nose_plane(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    """(x_ref, z_ref, slope): the cowl's top starts at the box front, level with the lid's top (the lid
    sits on the walls), and slopes down: z = z_ref - slope * (x - x_ref)."""
    _, x1, _, _ = frame.tray_box(p, fr)
    return x1, frame.box_top(p, fr, d) + fr.lid_t, b.slope


def plane_z(x, p: Params = P, b: BodyParams = B):
    x_ref, z1, k = nose_plane(p, b)
    return z1 - k * max(0.0, x - x_ref)


def _section(x, hw, depth, p, b, inset=0.0):
    """Wedge section: vertical sides, chamfered top corners, top on the nose plane."""
    zt = plane_z(x, p, b) - inset
    hw = hw - inset
    c = min(b.facet, hw * 0.4)
    zb = zt - depth - (1.0 if inset else 0.0)  # the inner cut runs out of the bottom (open underside)
    pts = [(-hw, zb), (hw, zb), (hw, zt - c), (hw - c, zt), (-hw + c, zt), (-hw, zt - c)]
    return Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * make_face(Polyline(*pts, close=True))


def wedge(p: Params = P, b: BodyParams = B, d=drive.D):
    """The cowl: a hollow faceted wedge, open underneath, closed by its front cut face."""
    _, x1, _, _ = frame.tray_box(p)
    x_ref, _, _ = nose_plane(p, b)
    st = [(x1 - 0.01 if i == 0 else x, hw, dep) for i, (x, hw, dep) in enumerate(b.stations)]
    if x_ref > st[0][0]:  # a station where the flat top turns into the slope
        (xa, hwa, da), (xb, hwb, db) = st[0], st[1]
        t = (x_ref - xa) / (xb - xa)
        st.insert(1, (x_ref, hwa + t * (hwb - hwa), da + t * (db - da)))
    body = loft([_section(x, hw, dep, p, b) for x, hw, dep in st], ruled=True)
    xe, hwe, depe = st[-1]
    try:  # soften the cut face's top edges (top plane and the two facets)
        front = [fc for fc in body.faces() if abs(fc.center().X - xe) < 1e-3][0]
        z_cut = plane_z(xe, p, b) - 2 * b.facet
        body = chamfer([e for e in front.edges() if e.center().Z > z_cut], b.front_c)
    except Exception:  # noqa: BLE001 - cosmetic only
        pass
    inner = st[:-1] + [(xe - b.skin, hwe, depe)]
    body -= loft([_section(x, hw, dep, p, b, inset=b.skin) for x, hw, dep in inner], ruled=True)
    return body


def top_body(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    x_ref, z1, k = nose_plane(p, b, d, fr)
    _, x1, _, _ = frame.tray_box(p, fr)
    zt = frame.box_top(p, fr, d)
    body = frame.frame(p)
    # everything in front of the box and within the cowl's width stops at the nose plane (trims the airbox
    # tube)
    fx, fy = fan.centre(p)
    w = 2 * b.stations[0][1]
    above = Pos(x_ref, 0, z1) * Rot(0, _deg(k), 0) * Box(200, w, 60, align=(Align.MIN, Align.CENTER, Align.MIN))
    above += Pos(x1, 0, z1) * Box(200, w, 60, align=(Align.MIN, Align.CENTER, Align.MIN))
    body -= above
    body += wedge(p, b, d)
    # room for the lid on the walls (0.1 all round), then the lip over its front edge
    body -= Pos(0, 0, zt) * extrude(offset(frame.lid_face(p, fr), 0.1), 10)
    # lip over the lid's front edge: the lid tucks under it, so only one screw is needed at the rear
    zl = zt + fr.lid_t
    body += Pos(x1 + 0.5, 0, zl) * Box(b.lip_d + 0.5, 2 * b.lip_hw, b.lip_t, align=(Align.MAX, Align.CENTER, Align.MAX))
    body -= above  # the lip's forward end follows the plane
    # airbox bore through everything (the fan motor pokes through the plane)
    h = fan.heights(p)
    r_in = p.motor.d / 2 + fr.tube_clear
    body -= Pos(fx, fy, h["plate"]) * Cylinder(r_in, 60, align=MIN)
    body.label, body.color = "body", (0.85, 0.12, 0.12)
    return body


def _deg(k):
    from math import atan, degrees
    return degrees(atan(k))


def lid(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    x0, x1, y0, y1 = frame.tray_box(p, fr)
    zt = frame.box_top(p, fr, d)
    bosses = frame.lid_bosses(p, fr)
    # plate on the walls, with a lug over the rear screw boss
    body = Pos(0, 0, zt) * extrude(frame.lid_face(p, fr), fr.lid_t)
    try:  # small chamfer on the top edge
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
        body -= Pos((x0 + x1) / 2 - 1.5, yc, zt) * Box(x1 - x0 - 2 * m - 5, gw, fr.lid_t, align=MIN)
    for bx, by in bosses:
        body -= Pos(bx, by, zt + fr.lid_t) * drive.countersunk(d, depth=5, up=10)
    # front tongue under the nose lip
    g = b.lip_gap
    body -= Pos(x1 + 1, 0, zt + fr.lid_t) * Box(b.lip_d + g + 1, 2 * (b.lip_hw + 2 * g), b.lip_t + g + 1,
                                                align=(Align.MAX, Align.CENTER, Align.MAX)).moved(Pos(0, 0, 1))
    # shark fin along the centre line, rising towards the rear wing
    z_top = zt + fr.lid_t
    wing_x = x0 + b.wing_chord / 2 + 0.5
    xf = x1 - 1.0
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
    return {"body": top_body(p), "lid": lid(p)}
