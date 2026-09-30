"""Top frame (FDM, PETG): battery box, posts onto the bearing-block caps, the fan "airbox" tube, and
the battery-box lid bosses. body.py adds the nose and front wing to make the one-piece top body.

The frame screws to the frame bosses on both caps, and the airbox tube screws to the fan mount's two
ears. The battery box walls hold the cells on every side; the lid holds them down.
"""

from dataclasses import dataclass
from build123d import (Align, Axis, Box, Cylinder, Plane, Polyline, Pos, RectangleRounded, extrude, fillet,
                       make_face)

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
    rib: float = 1.6  # ribs left between floor windows
    window: float = 7.0
    gill_w: float = 1.8  # vertical gill slots in the long walls
    gill_pitch: float = 3.6
    louvre_margin: float = 2.5  # solid wall kept at the top and bottom of the louvres
    wire_slot: tuple = (6.0, 5.0)  # (width, height) at the bottom of each end wall
    tube_wall: float = 1.2
    tube_clear: float = 0.3  # around the fan motor
    flange_t: float = 1.5  # tube flange screwed to the fan mount ears
    spine_h: float = 3.3  # rails stay 0.5 above the raised motor
    corner_r: float = 3.0  # rounded vertical corners of the battery box
    rib_pitch: float = 8.0  # floor ribs
    lid_boss_y: float = 14.0
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


def rail_y(p: Params = P, fr: FrameParams = FR):
    return p.motor.d / 2 + fr.tube_clear + fr.tube_wall - fr.wall_t / 2


def lid_bosses(p: Params = P, fr: FrameParams = FR):
    """(x, y) of the two lid screws: outside the rear wall (the front of the lid tucks under the nose)."""
    x0, _, _, _ = tray_box(p, fr)
    r = fr.boss_d / 2
    return (x0 - r + 0.3, fr.lid_boss_y), (x0 - r + 0.3, -fr.lid_boss_y)


def frame(p: Params = P, fr: FrameParams = FR, d=drive.D, f=fan.F):
    z0 = d.tray_bottom_z
    z1 = z0 + fr.floor_t
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
    # lid bosses outside the front and rear walls, insert at the top
    for bx, by in lid_bosses(p, fr):
        body += Pos(bx, by, zt - 6) * Cylinder(fr.boss_d / 2, 6, align=MIN)
        body -= Pos(bx, by, zt) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
        body -= Pos(bx, by, zt) * Cylinder(d.screw_clear_d / 2, d.screw_l - 0.9 + 0.5, align=MAX)  # lid 0.9 thick

    # posts onto the two cap bosses
    for side in (1, -1):
        bx, by, btop = drive.frame_boss(side, p, d)
        by *= side
        body += Pos(bx, by, z0) * Cylinder(fr.post_d / 2 + 0.6, fr.floor_t, align=MIN)
        if btop < z0:  # post down to a lower boss, screwed through its floor
            body += Pos(bx, by, btop) * Cylinder(fr.post_d / 2, z0 - btop, align=MIN)
            body -= Pos(bx, by, btop + fr.floor_t) * drive.countersunk(d, depth=5, up=40)
        else:
            body -= Pos(bx, by, z1) * drive.countersunk(d, depth=5, up=40)

    # airbox tube (body.py trims it to the nose plane), flange screwed to the fan mount ears
    h = fan.heights(p, f)
    fx, fy = fan.centre(p)
    collar_top = h["plate"] + f.plate_t + f.collar_h
    r_in = p.motor.d / 2 + fr.tube_clear
    r_out = r_in + fr.tube_wall
    body += Pos(fx, fy, collar_top) * Cylinder(r_out, zt - collar_top, align=MIN)
    er = f.ear_r
    body += Pos(fx, fy, collar_top) * Box(fr.boss_d, 2 * er + fr.boss_d, fr.flange_t, align=MIN)
    reach = er + fr.boss_d / 2
    for sy in (1, -1):
        body += Pos(fx, sy * er, collar_top) * Cylinder(fr.boss_d / 2, fr.flange_t, align=MIN)
        # 45 deg corbel on top of the flange: prints without support when the body is upside down
        zf = collar_top + fr.flange_t
        tri = make_face(Polyline((sy * (r_in + 0.5), zf), (sy * reach, zf), (sy * (r_in + 0.5), zf + reach - r_in - 0.5), close=True))
        body += Pos(fx - fr.boss_d / 2, 0, 0) * (Plane.YZ * extrude(tri, fr.boss_d))
        body -= Pos(fx, sy * er, zf) * drive.countersunk(d, depth=5, up=30)
    body -= Pos(fx, fy, collar_top - 1) * Cylinder(r_in, 40, align=MIN)
    # the fan motor's wires leave through the open airbox top

    body.label, body.color = "frame", (0.25, 0.25, 0.28)
    return body


def parts(p: Params = P):
    return {"frame": frame(p)}
