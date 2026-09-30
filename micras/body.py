"""Race-car styled body parts (FDM, PETG): halo, front wing, and the battery-box lid ("engine cover").

- Halo: an F1-style horseshoe hoop around the fan motor ("the driver"), screwed to the frame's spine
  rails at its two rear feet, with a central pillar sweeping down to a nose block.
- Front wing: main plane, endplates and a centre keel whose rounded bottom is the floor skid. The wing
  bears on the board's front edge, so crash loads go into the board; it screws to the nose block.
- Lid: the battery-box cover with louvres, a shark fin and a rear wing, held by two screws.
"""

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin

from build123d import (Align, Axis, Box, Cylinder, Ellipse, Plane, Polyline, Pos, Rot, extrude, loft,
                       make_face, revolve, Vector)

from . import drive, fan, frame
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class BodyParams:
    # halo
    hoop_z: float = 35.0  # hoop centre height (around the fan motor's top)
    hoop_w: float = 2.6  # hoop section, radial
    hoop_h: float = 3.4  # hoop section, vertical
    foot_d: float = 5.0
    pillar_w: float = 3.4  # pillar section across y
    pillar_d: float = 5.0  # pillar section in the bending plane (stiffness against the skid load)
    nose_block: tuple = (4.5, 6.0, 5.0)  # (x, y, z) block at the pillar's foot
    nose_block_z: float = 3.6  # block bottom above the board top (clears the front LEDs)
    # front wing
    edge_x: float = 53.5  # board front edge
    span: float = 24.0
    chord: float = 4.5
    plane_t: float = 0.9
    plane_z: float = 1.15  # main plane bottom (its rear edge bears on the board edge, z 1.0-2.04)
    endplate_t: float = 0.9
    endplate_h: float = 1.65  # stays below the sensor caps (z 2.74)
    keel_w: float = 6.0
    keel_len: float = 3.5  # in front of the board edge
    skid_z: float = 0.3  # skid bottom above the floor at rest
    skid_r: float = 1.5
    # lid
    lid_t: float = 0.9
    lid_margin: float = 1.6  # solid border and ribs
    louvre: tuple = (1.6, 3.4)  # (width, pitch) of the lid gills
    fin_t: float = 0.9
    fin_h: float = 6.0  # at the rear
    wing_span: float = 40.0
    wing_chord: float = 5.5
    wing_t: float = 0.9
    wing_z: float = 6.5  # above the lid top
    wing_angle: float = 12.0


B = BodyParams()


def _screw_z(p, b):
    return p.board.top_z + b.nose_block_z + b.nose_block[2] / 2


def halo(p: Params = P, b: BodyParams = B, d=drive.D, fr=frame.FR):
    fx, fy = fan.centre(p)
    r = fr.halo_r
    hx, hy = frame.halo_feet(p, fr)
    a_foot = degrees(atan2(hy, hx - fx))  # ~147 deg
    # hoop: elliptical section revolved from one foot, round the front, to the other
    section = Plane.XZ * Pos(r, b.hoop_z) * Ellipse(b.hoop_w / 2, b.hoop_h / 2)
    hoop = revolve(Rot(0, 0, -a_foot) * section, Axis.Z, 2 * a_foot)
    body = Pos(fx, fy, 0) * hoop
    # feet down to the spine rails, screwed from the top (head seat 3 mm above the rail top)
    z_rail = frame.D_TRAY_TOP(p, d, fr)
    for sy in (1, -1):
        body += Pos(hx, sy * hy, z_rail) * Cylinder(b.foot_d / 2, b.hoop_z - z_rail, align=MIN)
        body -= Pos(hx, sy * hy, z_rail + d.screw_l - d.insert_l) * drive.countersunk(d, depth=5, up=20)
    # nose block at the pillar's foot, insert facing forward for the wing screw
    nx, ny, nz = b.nose_block
    zb = p.board.top_z + b.nose_block_z
    front = b.edge_x - 0.1
    body += Pos(front - nx / 2, 0, zb) * Box(nx, ny, nz, align=MIN)
    zs = _screw_z(p, b)
    body -= Pos(front, 0, zs) * Rot(0, -90, 0) * Cylinder(d.insert_d / 2, d.insert_l, align=MIN)
    body -= Pos(front, 0, zs) * Rot(0, -90, 0) * Cylinder(d.screw_clear_d / 2, 3.5, align=MIN)
    # pillar: tapered elliptical loft from the hoop's front to the nose block
    top = Vector(fx + r, 0, b.hoop_z)
    bot = Vector(front - nx / 2, 0, zb + nz * 0.6)
    axis = (bot - top).normalized()
    s_top = Plane(origin=top, x_dir=(0, 1, 0), z_dir=axis) * Ellipse(b.pillar_w / 2, b.pillar_d / 2)
    s_bot = Plane(origin=bot, x_dir=(0, 1, 0), z_dir=axis) * Ellipse(b.pillar_w / 2 * 0.8, b.pillar_d / 2)
    body += loft([s_top, s_bot])
    body.label, body.color = "halo", (0.9, 0.9, 0.92)
    return body


def front_wing(p: Params = P, b: BodyParams = B, d=drive.D):
    x0 = b.edge_x
    nx, ny, nz = b.nose_block
    zb = p.board.top_z + b.nose_block_z
    zs = _screw_z(p, b)
    # main plane: rear edge on the board's front edge
    body = Pos(x0 + b.chord / 2, 0, b.plane_z) * Box(b.chord, b.span, b.plane_t, align=MIN)
    # endplates, swept forward and kept below the sensor caps
    for sy in (1, -1):
        body += Pos(x0 + b.chord / 2 + 0.6, sy * (b.span / 2 - b.endplate_t / 2), b.plane_z - 0.4) * Box(
            b.chord + 1.2, b.endplate_t, b.endplate_h, align=MIN)
    # nose cone: a tapered loft from behind the nose block to a rounded tip, hollow for the block
    tip_x = x0 + b.chord + 1.0
    sections = [
        (x0 - nx - 0.6, 0.5 * ny + 0.9, 0.5 * nz + 0.9, zb + nz / 2),
        (x0, 0.5 * ny + 0.8, 0.5 * nz + 0.9, zb + nz / 2 - 0.4),
        (tip_x, 1.2, 1.0, b.plane_z + b.plane_t + 1.0),
    ]
    faces = [Plane(origin=(x, 0, z), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * Ellipse(hw, hh) for x, hw, hh, z in sections]
    cone = loft(faces)
    cone -= Pos(x0 - nx / 2 - 0.1, 0, zb - 0.15) * Box(nx + 0.6, ny + 0.3, nz + 0.3, align=MIN)  # the halo's nose block
    cone -= Pos(x0 - nx - 2, 0, 0) * Box(4, 20, 30, align=(Align.MAX, Align.CENTER, Align.MIN))
    body += cone
    # keel under the cone with the rounded skid
    kz = b.skid_z + b.skid_r
    body += Pos(x0 + b.keel_len / 2, 0, kz) * Box(b.keel_len, b.keel_w * 0.6, zb - kz, align=MIN)
    body += Pos(x0 + b.keel_len / 2, 0, kz) * Rot(90, 0, 0) * Cylinder(b.skid_r, b.keel_w * 0.6)
    # screw through the cone into the nose block
    # head seat so the M2x5 reaches the bottom of the nose-block insert (block front face at x0 - 0.1)
    seat = x0 - 0.1 + d.screw_l - d.insert_l
    body -= Pos(seat, 0, zs) * Rot(0, 90, 0) * drive.countersunk(d, depth=d.screw_l, up=10)
    # nothing may go behind the board edge below the board top (the edge face is the contact)
    body -= Pos(x0, 0, 0) * Box(20, 100, p.board.top_z + 0.2, align=(Align.MAX, Align.CENTER, Align.MIN))
    body.label, body.color = "front_wing", (0.9, 0.9, 0.92)
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
    body.label, body.color = "lid", (0.9, 0.15, 0.15)
    return body


def parts(p: Params = P):
    return {"halo": halo(p), "front_wing": front_wing(p), "lid": lid(p)}
