"""Top frame (FDM, PETG): battery box, posts onto the bearing-block caps and the fan "airbox" tube.
body.py adds the nose cowl to make the one-piece top body.

The frame screws to the frame bosses on both caps; its airbox tube presses the fan mount down and clamps
the fan motor. The battery box walls hold the cells on every side. The lid sits on the walls and is
screwed on the centre line at both ends, into bosses blended into the front and rear walls (the front one
sits inside the nose; its lug lies flush in a pocket in the nose top).
"""

from dataclasses import dataclass
from build123d import Align, Box, Circle, Cone, Cylinder, Pos, RectangleRounded, extrude, offset

from . import drive, fan
from .mass import battery_cells
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FrameParams:
    floor_t: float = 1.2  # box floor (FDM: 6 layers at 0.2)
    wall_t: float = 0.9  # box walls (2 perimeters at 0.45)
    pack_clear: float = 0.3
    post_d: float = 5.6
    boss_d: float = 5.6  # around an M2 insert
    gill_w: float = 1.8  # vertical gill slots in the long walls
    gill_pitch: float = 3.6
    louvre_margin: float = 2.5  # solid wall kept at the top and bottom of the louvres
    wire_slot: tuple = (6.0, 5.0)  # (width, height) at the bottom of each end wall
    tube_wall: float = 1.2
    tube_clear: float = 0.3  # around the fan motor
    collet_squeeze: float = 0.1  # radial interference between the airbox taper and the fan-mount collar
    corner_r: float = 1.9  # outer corners of the box (inner 1.0: clears the cells' square corners)
    rib_pitch: float = 8.0  # floor ribs
    lid_t: float = 0.9
    lug_t: float = 1.5  # the lid's screw lugs are thicker (they sink into the boss tops): the countersink
    # needs 0.9 of it
    insert_d: float = 3.1  # heat-set insert hole in PETG (M2x2, OD 3.2)
    lid_boss_off: float = 1.9  # lid screw axes outside the front and rear walls (0.8 PETG to the cells)
    floor_band: float = 6.0  # solid floor band under each body screw (the rest of the floor is ribs)
    lid_boss_h: float = 5.0  # full-radius height below the box top (insert + screw tip), then a 45 deg taper
    blend_r: float = 1.5  # fillet between the boss and the wall (plan view)
    gills: bool = False  # vertical gill slots in the box walls (lighter; off for the clean faceted look)


FR = FrameParams()


def D_TRAY_TOP(p: Params = P, d=drive.D, fr: "FrameParams" = None):
    """Top of the box floor."""
    return d.tray_bottom_z + (fr or FR).floor_t


def pack_size(p: Params = P):
    """(x, y, z) size of the battery pack."""
    _, (sx, sz) = battery_cells(p.battery.arrangement, p)
    L, W, T = p.battery.cell
    sy = W if p.battery.arrangement == "standing" else L
    return sx, sy, sz


def tray_box(p: Params = P, fr: FrameParams = FR):
    """(x0, x1, y0, y1) of the box outline."""
    sx, sy, _ = pack_size(p)
    m = fr.pack_clear + fr.wall_t
    return p.battery.x - sx / 2 - m, p.battery.x + sx / 2 + m, -sy / 2 - m, sy / 2 + m


def box_top(p: Params = P, fr: FrameParams = FR, d=drive.D):
    return D_TRAY_TOP(p, d, fr) + pack_size(p)[2] + fr.pack_clear


def cavity(p: Params = P, fr: FrameParams = FR):
    """(x0, x1, y0, y1) of the inside of the box."""
    x0, x1, y0, y1 = tray_box(p, fr)
    return x0 + fr.wall_t, x1 - fr.wall_t, y0 + fr.wall_t, y1 - fr.wall_t


def lid_bosses(p: Params = P, fr: FrameParams = FR):
    """(x, y) of the lid screws: on the centre line, in front of the front wall and behind the rear wall."""
    x0, x1, _, _ = tray_box(p, fr)
    return [(x1 + fr.lid_boss_off, 0.0), (x0 - fr.lid_boss_off, 0.0)]


def _plan_with_bosses(p: Params = P, fr: FrameParams = FR):
    """Box outline with the lid boss blended in (plan fillets)."""
    x0, x1, y0, y1 = tray_box(p, fr)
    plan = Pos((x0 + x1) / 2, 0) * RectangleRounded(x1 - x0, y1 - y0, fr.corner_r)
    for bx, by in lid_bosses(p, fr):
        plan += Pos(bx, by) * Circle(fr.boss_d / 2)
    return offset(offset(plan, fr.blend_r), -fr.blend_r)


def lid_face(p: Params = P, fr: FrameParams = FR):
    """Plan of the lid: the box outline with a lug over the screw boss."""
    return _plan_with_bosses(p, fr)


def frame(p: Params = P, fr: FrameParams = FR, d=drive.D, f=fan.F):
    z0 = d.tray_bottom_z
    z1 = z0 + fr.floor_t
    assert abs(z1 - p.battery.floor_z) < 1e-6, "Battery.floor_z must equal tray_bottom_z + floor_t"
    zt = box_top(p, fr, d)
    x0, x1, y0, y1 = tray_box(p, fr)
    cx = (x0 + x1) / 2

    # box floor: ribs running front to back (no plate, so no ceiling when printed on the nose plane);
    # the cells rest on the rib edges
    body = Pos(cx, 0, z0) * extrude(RectangleRounded(x1 - x0, y1 - y0, fr.corner_r), fr.floor_t)
    body -= Pos(cx, 0, z0) * extrude(RectangleRounded(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t,
                                                      fr.corner_r - fr.wall_t), fr.floor_t)
    n_rib = int((y1 - y0) // fr.rib_pitch)
    for i in range(n_rib):
        yc = (i - (n_rib - 1) / 2) * fr.rib_pitch
        body += Pos(cx, yc, z0) * Box(x1 - x0 - fr.wall_t, fr.wall_t, fr.floor_t, align=MIN)
    # full-height walls on all four sides
    outer = Pos(cx, 0, z1) * extrude(RectangleRounded(x1 - x0, y1 - y0, fr.corner_r), zt - z1)
    inner = Pos(cx, 0, z1) * extrude(RectangleRounded(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t,
                                                      fr.corner_r - fr.wall_t), zt - z1)
    body += outer - inner
    # lid screw boss blended into the rear wall, tapering into it below
    zp = zt - fr.lid_boss_h
    body += Pos(0, 0, zp) * extrude(_plan_with_bosses(p, fr), zt - zp) - inner
    for bx, by in lid_bosses(p, fr):
        body += (Pos(bx, by, zp) * Cone(0.01, fr.boss_d / 2, fr.boss_d / 2, align=MAX)) - inner
    # gills in the front and rear walls (lighter, and the race-car look)
    lh = zt - z1 - 2 * fr.louvre_margin
    n = int((y1 - y0 - 6) // fr.gill_pitch) if fr.gills else 0
    for i in range(n):
        yc = (i - (n - 1) / 2) * fr.gill_pitch
        if min(abs(yc - by) for _, by in lid_bosses(p, fr)) < fr.boss_d / 2 + fr.gill_w:  # solid behind the lid bosses
            continue
        for x in (x0, x1):
            body -= Pos(x, yc, z1 + fr.louvre_margin) * Box(3 * fr.wall_t, fr.gill_w, lh, align=MIN)
    # wire slots at the bottom of both end walls
    sw, sh = fr.wire_slot
    for y in (y0, y1):
        body -= Pos(x0 + sw / 2 + fr.wall_t + 1, y, z1) * Box(sw, 3 * fr.wall_t, sh, align=MIN)
    # lid inserts, below a pocket for the thick lug
    sink = fr.lug_t - fr.lid_t
    for bx, by in lid_bosses(p, fr):
        body -= Pos(bx, by, zt) * Cylinder(fr.boss_d / 2 - 0.3, sink, align=MAX)
        body -= Pos(bx, by, zt - sink) * Cylinder(fr.insert_d / 2, d.insert_l, align=MAX)
        body -= Pos(bx, by, zt - sink) * Cylinder(d.screw_clear_d / 2, d.screw_l - fr.lug_t + 0.5, align=MAX)

    # posts onto the two cap bosses, each on a solid floor band that spans the box (front wall to rear wall)
    for side in (1, -1):
        bx, by, btop = drive.frame_boss(side, p, d)
        by *= side
        body += Pos(cx, by, z0) * Box(x1 - x0 - fr.wall_t, fr.floor_band, fr.floor_t, align=MIN)
        body += Pos(bx, by, z0) * Cylinder(fr.post_d / 2 + 0.6, fr.floor_t, align=MIN)
        if btop < z0:  # post down to a lower boss, screwed through its floor
            body += Pos(bx, by, btop) * Cylinder(fr.post_d / 2, z0 - btop, align=MIN)
            body -= Pos(bx, by, btop + fr.floor_t) * drive.countersunk(d, depth=5, up=40)
        else:
            body -= Pos(bx, by, z1) * drive.countersunk(d, depth=5, up=40)

    # airbox tube (body.py trims it to the nose plane). Its tapered bottom slides over the fan mount's
    # slotted collar top: when the body is screwed down it holds the mount on the board and squeezes the
    # collar onto the motor (a collet), so neither needs screws.
    h = fan.heights(p, f)
    fx, fy = fan.centre(p)
    rc = fan.collar_r(p, f)
    ct = h["collar_top"]
    zb = ct - f.taper_l
    r_in = p.motor.d / 2 + fr.tube_clear
    r_out = rc + fr.tube_wall
    body += Pos(fx, fy, zb) * Cylinder(r_out, zt - zb, align=MIN)
    body -= Pos(fx, fy, zb) * Cone(rc - fr.collet_squeeze, rc - f.taper - fr.collet_squeeze, f.taper_l, align=MIN)
    body -= Pos(fx, fy, zb - 1) * Cylinder(r_in, 60, align=MIN)
    # the fan motor's wires leave through the open airbox top

    body.label, body.color = "frame", (0.25, 0.25, 0.28)
    return body


def parts(p: Params = P):
    return {"frame": frame(p)}
