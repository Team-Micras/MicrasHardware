"""Battery basket (FDM, PETG): an open box on two posts onto the bearing-block caps, the cells held in by two
velcro straps over the top.

The floor is ribs plus a solid band under each screw post and a strip along each side wall (tools/fea.py sized
them for a 300 g drop and a 100 g side crash). The front and rear walls hold the cells fore and aft and carry the
straps: each strap runs over the pack and through a window in each wall, and closes on itself. The side walls
are low: the cells' ends only need locating.
"""

from dataclasses import dataclass

from build123d import Align, Box, Cone, Cylinder, Pos, RectangleRounded, extrude

from . import drive
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
    post_flare: float = 1.5  # 45 deg flare where a post meets the floor
    wire_slot: tuple = (6.0, 5.0)  # (width, height) at the rear of each side wall
    corner_r: float = 1.9  # outer corners of the box (inner 1.0: clears the cells' square corners)
    rib_pitch: float = 8.0  # floor ribs
    floor_band: float = 9.0  # solid floor band under each screw post (the rest of the floor is ribs; tools/fea.py)
    side_band: float = 2.5  # solid floor strip inside each side wall (it holds the wall's foot in a side crash)
    side_wall_h: float = 5.0  # the long cells' ends only need locating; the straps hold them down
    rim: float = 1.8  # front and rear walls above the pack top (the strap windows sit in this band)
    strap_w: float = 10.0  # velcro strap
    strap_t: float = 0.8
    strap_y: float = 12.0  # |y| of the two straps


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


def floor(p: Params = P, fr: FrameParams = FR, d=drive.D):
    """Box floor (ribs, the bands under the screws, side strips), the outline rim, and the posts onto the two
    cap bosses with their countersunk screws."""
    z0 = d.tray_bottom_z
    z1 = z0 + fr.floor_t
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
    for sy in (1, -1):
        body += Pos(cx, sy * (y1 - fr.wall_t), z0) * Box(x1 - x0 - fr.wall_t, fr.side_band, fr.floor_t,
                                                     align=(Align.CENTER, Align.MAX if sy > 0 else Align.MIN, Align.MIN))
    # posts onto the two cap bosses, each on a solid floor band that spans the box (front wall to rear wall)
    for side in (1, -1):
        bx, by, btop = drive.frame_boss(side, p, d)
        by *= side
        body += Pos(cx, by, z0) * Box(x1 - x0 - fr.wall_t, fr.floor_band, fr.floor_t, align=MIN)
        body += Pos(bx, by, z0) * Cylinder(fr.post_d / 2 + 0.6, fr.floor_t, align=MIN)
        if btop < z0:  # post down to a lower boss, screwed through its floor
            body += Pos(bx, by, btop) * Cylinder(fr.post_d / 2, z0 - btop, align=MIN)
            # flared into the floor (the post's foot was the peak stress in a drop, tools/fea.py)
            body += Pos(bx, by, z0) * Cone(fr.post_d / 2, fr.post_d / 2 + fr.post_flare, fr.post_flare, align=MAX)
            body -= Pos(bx, by, btop + fr.floor_t) * drive.countersunk(d, depth=5, up=40)
        else:
            body -= Pos(bx, by, z1) * drive.countersunk(d, depth=5, up=40)
    return body


def basket(p: Params = P, fr: FrameParams = FR, d=drive.D):
    """The floor and posts, low side walls, full front and rear walls with the strap windows."""
    z1 = d.tray_bottom_z + fr.floor_t
    assert abs(z1 - p.battery.floor_z) < 1e-6, "Battery.floor_z must equal tray_bottom_z + floor_t"
    zt = box_top(p, fr, d)
    x0, x1, y0, y1 = tray_box(p, fr)
    cx = (x0 + x1) / 2
    body = floor(p, fr, d)
    outer = RectangleRounded(x1 - x0, y1 - y0, fr.corner_r)
    inner = RectangleRounded(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t, fr.corner_r - fr.wall_t)
    walls = Pos(cx, 0, z1) * extrude(outer - inner, zt + fr.rim - z1)
    for s in (1, -1):  # low side walls
        walls -= Pos(cx, s * (y1 - fr.wall_t / 2), z1 + fr.side_wall_h) * Box(
            x1 - x0 - 2 * fr.corner_r, 3 * fr.wall_t, 40, align=MIN)
    body += walls
    # strap windows in the front and rear walls, their bottom at the pack top (the strap presses on the cells)
    for s in (1, -1):
        body -= Pos(cx, s * fr.strap_y, zt - 0.3) * Box(x1 - x0 + 2, fr.strap_w + 0.6, fr.strap_t + 0.4, align=MIN)
    sw, sh = fr.wire_slot  # the battery wires leave at the rear
    for y in (y0, y1):
        body -= Pos(x0 + sw / 2 + fr.wall_t + 1, y, z1) * Box(sw, 3 * fr.wall_t, sh, align=MIN)
    body.label, body.color = "basket", (0.25, 0.25, 0.28)
    return body


def straps(p: Params = P, fr: FrameParams = FR, d=drive.D):
    """The two velcro straps (bought): over the pack, through the windows, closed on themselves on top."""
    zt = box_top(p, fr, d)
    x0, x1, _, _ = tray_box(p, fr)
    out = {}
    for i, s in enumerate((1, -1)):
        band = Pos((x0 + x1) / 2, s * fr.strap_y, zt - 0.1) * Box(x1 - x0 + 2.4, fr.strap_w, fr.strap_t, align=MIN)
        band += Pos((x0 + x1) / 2 + 1, s * fr.strap_y, zt + fr.strap_t - 0.1) * Box(x1 - x0 - 2, fr.strap_w,
                                                                                   fr.strap_t, align=MIN)
        band.label, band.color = f"velcro_{i}", (0.1, 0.1, 0.1)
        out[band.label] = band
    return out


def parts(p: Params = P):
    return {"basket": basket(p)}
