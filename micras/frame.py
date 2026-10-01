"""Battery basket (FDM, PLA): an open box on two posts onto the bearing-block caps, the cells held in by two
velcro straps over the top.

The floor is ribs plus a solid band under each screw post and a strip along each side wall (tools/fea.py sized
them for a 300 g drop and a 100 g side crash). The walls hold the cells on every side; the front and rear walls carry the
straps: each strap runs over the pack and through a window in each wall, and closes on itself. Saddle ribs under
the floor rest along the drive caps' top edges, so the basket can't pitch about its two posts.
"""

from dataclasses import dataclass, replace

from build123d import Align, Box, Cone, Cylinder, Plane, Polygon, Pos, RectangleRounded, Rot, SlotOverall, extrude

from . import drive
from .mass import battery_cells
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FrameParams:
    floor_t: float = 1.2  # box floor (FDM: 6 layers at 0.2)
    floor_skin: float = 0.4  # solid skin on the floor's top (the first bridge when printed upside down)
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
    # walls' height above the floor: low walls only locate the cells, and a strap front to back holds them,
    # through a slotted lug on the foot of the front and rear walls.
    # None: full-height walls, two straps front to back through windows in the front and rear walls
    wall_h: float = 4.0
    strap_w: float = 10.0  # velcro strap
    strap_t: float = 0.8
    lug: float = 4.0  # 45 deg ramp on the wall's foot, outside, ending in a lug_end tall face: the strap goes down
    # through its slot, under its outer part (the bar) and back up
    lug_end: float = 1.6
    lug_slot: tuple = (0.8, 1.2)  # slot's distance from the wall, and its width
    # full walls only: two straps front to back at these |y|, and the walls' feet thickened outside (a crash
    # throws the cells at the tall front wall and bends it at its foot, across the layers: tools/fea.py)
    strap_y: float = 12.0
    rim: float = 1.8  # above the pack top (the strap windows sit in this band)
    foot_t: float = 0.6
    foot_h: float = 7.0
    # feet under the floor resting on the drive caps (drive.DriveParams.basket_feet): with the two posts they
    # keep the basket from pitching in a crash or a drop (tools/fea.py)
    foot_d: float = 4.0
    foot_gap: float = 0.1  # above the cap's top (prints to a resting contact)
    peg_d: float = 1.9  # into the cap's socket (drive.DriveParams.socket_d)


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
    # box floor: ribs running front to back, with a thin skin on the cells' side: the basket prints upside down,
    # and the skin is then one bridge across the box between the end walls (the ribs alone would start as
    # strips in mid-air); the cells rest on it
    body = Pos(cx, 0, z0) * extrude(RectangleRounded(x1 - x0, y1 - y0, fr.corner_r), fr.floor_t)
    body -= Pos(cx, 0, z0) * extrude(RectangleRounded(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t,
                                                      fr.corner_r - fr.wall_t), fr.floor_t)
    body += Pos(cx, 0, z1 - fr.floor_skin) * extrude(RectangleRounded(x1 - x0 - fr.wall_t, y1 - y0 - fr.wall_t,
                                                                      fr.corner_r - fr.wall_t / 2), fr.floor_skin)
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


def _feet(p: Params = P, fr: FrameParams = FR, d=drive.D):
    """Columns from the floor down onto the caps, their undersides following the cap's top (with a peg for the
    cap's socket where the foot has one)."""
    out = None
    # (from the default blocks: their tops are the same in every variant, so one basket fits them all)
    pd = replace(p, layout=replace(p.layout, backlash_mode="eccentric", blocks="split"))
    caps = {side: drive.block(side, pd, d)[-1] for side in {fs for fs, _, _ in d.basket_feet}}
    for side, x, kind in d.basket_feet:
        cap = caps[side]
        y = (1 if side > 0 else -1) * drive.plate_mid_y(side, p, d)
        probe = cap & Pos(x, y, 0) * Cylinder(fr.foot_d / 2, 100, align=MIN)
        bb = probe.bounding_box()
        z_lo = bb.max.Z - 3
        foot = Pos(x, y, z_lo) * Cylinder(fr.foot_d / 2, d.tray_bottom_z + 0.01 - z_lo, align=MIN)
        # everything under the cap's top within the footprint (the plate has pockets on its inboard face)
        shadow = probe
        for k in range(1, 12):
            shadow = shadow.fuse(Pos(0, 0, -0.4 * k) * probe)
        foot -= Pos(0, 0, fr.foot_gap) * shadow
        if kind == "peg":  # down to 0.2 above the socket's floor (the cap's highest point at the socket's axis)
            floor_z = (cap & Pos(x, y, 0) * Box(0.2, 0.2, 100, align=MIN)).bounding_box().max.Z
            foot += Pos(x, y, floor_z + 0.2) * Cylinder(fr.peg_d / 2, d.socket_depth + 1, align=MIN)
        solids = sorted(foot.solids(), key=lambda so: -so.volume)
        foot = solids[0] if len(solids) == 1 else solids[0].fuse(*solids[1:]) if kind == "peg" else solids[0]
        out = foot if out is None else out + foot
    return out


def pack_top(p: Params = P, fr: FrameParams = FR, d=drive.D):
    return D_TRAY_TOP(p, d, fr) + pack_size(p)[2]


def _lug(fr: FrameParams, width):
    """Wedge on a wall's foot, outward along +x from x=0, its slot cut through: local frame, floor bottom at z=0."""
    prof = Polygon((0, 0), (fr.lug, 0), (fr.lug, fr.lug_end), (0, fr.lug_end + fr.lug), align=None)
    lug = extrude(Plane.XZ * prof, width / 2, both=True)
    s0, sw = fr.lug_slot  # round-ended (square ends concentrate the strap's pull)
    lug -= Pos(s0 + sw / 2, 0, -1) * extrude(SlotOverall(fr.strap_w + 0.6, sw, rotation=90), 20)
    return lug


def basket(p: Params = P, fr: FrameParams = FR, d=drive.D):
    """The floor and posts, walls (low with strap lugs, or full height with strap windows), saddle ribs."""
    z0 = d.tray_bottom_z
    z1 = z0 + fr.floor_t
    assert abs(z1 - p.battery.floor_z) < 1e-6, "Battery.floor_z must equal tray_bottom_z + floor_t"
    zt = box_top(p, fr, d)
    x0, x1, y0, y1 = tray_box(p, fr)
    cx = (x0 + x1) / 2
    body = floor(p, fr, d)
    outer = RectangleRounded(x1 - x0, y1 - y0, fr.corner_r)
    inner = RectangleRounded(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t, fr.corner_r - fr.wall_t)
    top = z1 + fr.wall_h if fr.wall_h is not None else zt + fr.rim
    body += Pos(cx, 0, z1) * extrude(outer - inner, top - z1)
    body += _feet(p, fr, d)
    if fr.wall_h is not None:
        # strap lugs: front and rear, on the centre line
        lug = _lug(fr, fr.strap_w + 4.0)
        body += Pos(x1 - 0.01, 0, z0) * lug + Pos(x0 + 0.01, 0, z0) * lug.mirror(Plane.YZ)
        # the lugs' ramps end level with the wall tops: the basket prints upside down on them
        body -= Pos(cx, 0, top) * Box(200, 200, 20, align=MIN)
    else:
        foot = Polygon((0, z0), (fr.foot_t, z0), (fr.foot_t, z1 + fr.foot_h), (0, z1 + fr.foot_h + fr.foot_t),
                       align=None)
        foot = extrude(Plane.XZ * foot, (y1 - y0) / 2 - fr.corner_r, both=True)
        body += Pos(x1 - 0.01, 0, 0) * foot + Pos(x0 + 0.01, 0, 0) * foot.mirror(Plane.YZ)
        # strap windows in the front and rear walls, their bottom at the pack top (the strap presses on the cells)
        for s in (1, -1):
            body -= Pos(cx, s * fr.strap_y, zt - 0.3) * Box(x1 - x0 + 2, fr.strap_w + 0.6, fr.strap_t + 0.4, align=MIN)
        sw, sh = fr.wire_slot  # the battery wires leave at the rear (over the low walls otherwise)
        for y in (y0, y1):
            body -= Pos(x0 + sw / 2 + fr.wall_t + 1, y, z1) * Box(sw, 3 * fr.wall_t, sh, align=MIN)
    body.label, body.color = "basket", (0.25, 0.25, 0.28)
    return body


def straps(p: Params = P, fr: FrameParams = FR, d=drive.D):
    """The two velcro straps (bought), for the pictures and the mass."""
    x0, x1, y0, y1 = tray_box(p, fr)
    cx = (x0 + x1) / 2
    out = {}
    if fr.wall_h is None:  # both front to back, through the windows, closed on themselves on top
        zt = box_top(p, fr, d)
        for i, s in enumerate((1, -1)):
            band = Pos(cx, s * fr.strap_y, zt - 0.1) * Box(x1 - x0 + 2.4, fr.strap_w, fr.strap_t, align=MIN)
            band += Pos(cx + 1, s * fr.strap_y, zt + fr.strap_t - 0.1) * Box(x1 - x0 - 2, fr.strap_w, fr.strap_t,
                                                                             align=MIN)
            band.label, band.color = f"velcro_{i}", (0.1, 0.1, 0.1)
            out[band.label] = band
        return out
    # over the pack front to back, down outside the walls, through the lugs' slots (closed on itself on top)
    z0, zp = d.tray_bottom_z, pack_top(p, fr, d)
    s0, sw = fr.lug_slot
    t = fr.strap_t
    o0, o1 = x0 - s0 - sw / 2, x1 + s0 + sw / 2  # down through the slots' middles
    band = Pos((o0 + o1) / 2, 0, zp) * Box(o1 - o0 + t, fr.strap_w, t, align=MIN)
    for o in (o0, o1):
        band += Pos(o, 0, z0 - t) * Box(t, fr.strap_w, zp + t - (z0 - t), align=MIN)
    band.label, band.color = "velcro_0", (0.1, 0.1, 0.1)
    return {band.label: band}


def parts(p: Params = P):
    return {"basket": basket(p)}
