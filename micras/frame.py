"""Battery basket (FDM, PETG): an open box on two posts onto the bearing-block caps, the cells held in by two
velcro straps over the top.

The floor is ribs plus a solid band under each screw post and a strip along each side wall (tools/fea.py sized
them for a 300 g drop and a 100 g side crash). The walls hold the cells on every side; the front and rear walls carry the
straps: each strap runs over the pack and through a window in each wall, and closes on itself. Saddle ribs under
the floor rest along the drive caps' top edges, so the basket can't pitch about its two posts.
"""

from dataclasses import dataclass

from build123d import Align, Box, Cone, Cylinder, Plane, Polygon, Pos, RectangleRounded, extrude

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
    end_band: float = 3.0  # and inside the front and rear walls (else the screw bands' edges peak in a crash)
    side_wall_h: float = None  # side walls' height above the floor (None: full height; low walls let the front
    # and rear walls bend out in a crash, tools/fea.py)
    rim: float = 1.8  # front and rear walls above the pack top (the strap windows sit in this band)
    strap_w: float = 10.0  # velcro strap
    strap_t: float = 0.8
    strap_y: float = 12.0  # |y| of the two straps
    # the front and rear walls are thicker at the foot, outside (a crash throws the cells at them and bends
    # them there, across the layers: tools/fea.py), with a 45 deg top edge (no overhang printed upside down)
    foot_t: float = 0.6
    foot_h: float = 7.0  # above the floor
    saddle_w: float = 2.0  # rib under the floor resting along each drive cap's top edge (stops the basket
    # pitching about its two posts in a crash or a drop: tools/fea.py)
    saddle_gap: float = 0.1  # above the cap's top (prints to a resting contact)
    saddle_step: float = 0.5  # sampling of the cap's top edge
    saddle_min: float = 1.0  # the rib stops where it would be thinner than this (a sliver concentrates stress)


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
    for sx, xw in ((1, x1), (-1, x0)):
        body += Pos(xw - sx * fr.wall_t, 0, z0) * Box(fr.end_band, y1 - y0 - fr.wall_t, fr.floor_t,
                                                  align=(Align.MAX if sx > 0 else Align.MIN, Align.CENTER, Align.MIN))
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


def _saddle(side, x0, x1, p: Params = P, fr: FrameParams = FR, d=drive.D):
    """Rib under the floor over the cap's outer plate, its underside following the cap's top edge (sampled
    along x, taking the highest point across the rib's width), clear of the cap screws' heads."""
    s = 1 if side > 0 else -1
    _, cap = drive.block(side, p, d)
    rp = d.insert_d / 2 + d.pillar_wall
    y_mid = (drive.cap_screws(side, p, d)[0][1] - rp + drive.housing_span(p)[1]) / 2
    z_floor = d.tray_bottom_z
    tops = []
    n = int((x1 - x0) / fr.saddle_step)
    for i in range(n + 1):
        x = x0 + i * (x1 - x0) / n
        slab = cap & Pos(x, s * y_mid, 0) * Box(fr.saddle_step, fr.saddle_w, 100, align=MIN)
        tops.append((x, slab.bounding_box().max.Z if slab is not None and slab.volume > 1e-4 else None))
    ribs = None
    run = []
    for x, z in tops + [(None, None)]:  # one rib per stretch of the cap under the floor
        if z is not None and z + fr.saddle_gap < z_floor - fr.saddle_min:
            run.append((x, z + fr.saddle_gap))
            continue
        if len(run) >= 3:
            pts = [(run[0][0], z_floor + 0.01)] + run + [(run[-1][0], z_floor + 0.01)]
            rib = extrude(Plane.XZ.offset(-s * y_mid) * Polygon(*pts, align=None), fr.saddle_w / 2, both=True)
            ribs = rib if ribs is None else ribs + rib
        run = []
    if ribs is None:
        return None
    for sx, sy in drive.cap_screws(side, p, d):  # clear of the screw heads (and a screwdriver)
        ribs -= Pos(sx, s * sy, 0) * Cylinder(d.csk_d / 2 + 0.4, 100, align=MIN)
    return ribs


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
    for s in (1, -1) if fr.side_wall_h is not None else ():  # low side walls
        walls -= Pos(cx, s * (y1 - fr.wall_t / 2), z1 + fr.side_wall_h) * Box(
            x1 - x0 - 2 * fr.corner_r, 3 * fr.wall_t, 40, align=MIN)
    body += walls
    z0 = d.tray_bottom_z
    foot = Polygon((0, z0), (fr.foot_t, z0), (fr.foot_t, z1 + fr.foot_h), (0, z1 + fr.foot_h + fr.foot_t), align=None)
    foot = extrude(Plane.XZ * foot, (y1 - y0) / 2 - fr.corner_r, both=True)
    body += Pos(x1 - 0.01, 0, 0) * foot + Pos(x0 + 0.01, 0, 0) * foot.mirror(Plane.YZ)
    for side in (1, -1):
        rib = _saddle(side, x0 + fr.wall_t, x1 - fr.wall_t, p, fr, d)
        if rib is not None:
            body += rib
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
