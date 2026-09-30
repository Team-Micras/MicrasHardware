"""Alternative layout, to compare looks and mass: no car body.

- The fan motor hangs from the drive blocks: a yoke clamps the motor in a split collar (M2 screw + nut) and
  reaches back with two arms to ears on the front of the two caps (M2 screw + trapped nut each). The fan's
  feet leave the board.
- The cells sit in an open basket (the same floor, posts and screws as the battery box) with two velcro straps
  over the top, threaded through windows in its front and rear walls. No lid, no nose.

Everything else (drive, front, gears) is the main design. tools/render.py --extra micras.alt:parts shows it.
"""

from dataclasses import dataclass, replace
from math import hypot

from build123d import (Align, Box, Circle, Cylinder, Plane, Polygon, Pos, RectangleRounded, RegularPolygon, Rot,
                       extrude, make_hull)

from . import drive, fan, frame, front
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class AltParams:
    # yoke tabs, on the cap ears (x, |y|): in front of the caps' front screws, clear of their heads
    tab: tuple = (10.2, 14.2)
    tab_r: float = 2.5
    tab_t: float = 1.8  # yoke tab thickness
    ear_t: float = 2.4  # cap ear thickness (a nut trap in its underside)
    arm_w: float = 2.6
    arm_z0: float = 11.0  # arm underside at the collar (it rises to the ear at the tab)
    nut_af: float = 4.0  # M2 nut across flats (+ fit)
    nut_t: float = 1.6
    clamp_slit: float = 0.8
    clamp_ear_t: tuple = (1.6, 2.2)  # head side, nut side (the nut sits in the thicker one)
    # basket
    side_wall_h: float = 5.0  # the long cells' ends only need locating; the straps hold them down
    rim: float = 1.8  # front / rear walls above the pack top (the strap windows sit in this band)
    strap_w: float = 10.0
    strap_t: float = 0.8
    strap_y: float = 12.0  # |y| of the two straps


A = AltParams()


def _ear_top(p=P, d=drive.D):
    """Top of the cap ears: level with the left cap's front screw column."""
    return p.axle_z + p.bearing.od / 2 + d.wall


def _hull(circles):
    edges = []
    for x, y, r in circles:
        edges += (Pos(x, y) * Circle(r)).edges()
    return make_hull(edges).face()


def _nut(af, t):
    return extrude(RegularPolygon(af / 3 ** 0.5, 6), t)


def block_cap(side, p: Params = P, a: AltParams = A, d=drive.D):
    """The main design's cap plus an ear in front of its front screw column, with a nut trap underneath."""
    _, cap = drive.block(side, p, d)
    sx, sy = drive.cap_screws(side, p, d)[0]
    s = 1 if side > 0 else -1
    tx, ty = a.tab
    zt = _ear_top(p, d)
    rp = d.insert_d / 2 + d.pillar_wall
    ear = Pos(0, 0, zt - a.ear_t) * extrude(_hull([(sx, s * sy, rp - 0.05), (tx, s * ty, a.tab_r)]), a.ear_t)
    cap = cap.fuse(ear).clean()
    cap -= Pos(tx, s * ty, zt - a.ear_t - 1) * Cylinder(d.screw_clear_d / 2, a.ear_t + 2, align=MIN)
    cap -= Pos(tx, s * ty, zt - a.ear_t - 0.01) * _nut(a.nut_af, a.nut_t + 0.01)
    # keep the front cap screw reachable (its counterbore runs up through the ear)
    seat = p.axle_z + d.screw_l - d.insert_l
    cap -= Pos(sx, s * sy, seat) * drive.countersunk(d, depth=0.1)
    cap.label = f"block_cap_{'L' if side > 0 else 'R'}"
    return cap


def fan_yoke(p: Params = P, a: AltParams = A, f=fan.F, d=drive.D):
    """Collar (as the fan mount's, without feet or collet) split at the front and clamped by an M2 screw
    across two ears, and two arms back to the cap ears."""
    cx, cy = fan.centre(p)
    h = fan.heights(p, f)
    rc = fan.collar_r(p, f)
    ct = h["collar_top"]
    body = fan.mount(p, replace(f, feet=(), slit_angles=(), taper=0.0))
    zt = _ear_top(p, d)
    ztab = zt + a.tab_t
    tx, ty = a.tab
    for s in (1, -1):
        # arm: a web in its own vertical plane, from the collar's side (low) up to the tab
        x0, y0 = cx + (rc - 0.5) * (tx - cx) / hypot(tx - cx, s * ty - cy), cy + (rc - 0.5) * (s * ty - cy) / hypot(tx - cx, s * ty - cy)
        length = hypot(tx - x0, s * ty - y0)
        side = Polygon((0, a.arm_z0), (length, zt), (length, ztab), (0, ztab), align=None)
        plane = Plane(origin=(x0, y0, 0), x_dir=(tx - x0, s * ty - y0, 0), z_dir=(s * ty - y0, -(tx - x0), 0))
        body += extrude(plane * side, a.arm_w / 2, both=True)
        body += Pos(tx, s * ty, zt) * Cylinder(a.tab_r, a.tab_t, align=MIN)
        body -= Pos(tx, s * ty, ztab) * drive.countersunk(d, depth=5, up=10)
    # split clamp at the front: a slit through the collar wall and two ears with a crosswise M2 screw
    t1, t2 = a.clamp_ear_t
    ear_x, ear_z = rc + 2.2, ct - 2.6
    zc0 = h["motor"]
    body += Pos(cx + rc - 0.6, cy - a.clamp_slit / 2 - t1, zc0) * Box(5.4, t1 + a.clamp_slit + t2, ct - zc0,
                                                                     align=(Align.MIN, Align.MIN, Align.MIN))
    body -= Pos(cx + rc - 2, cy, zc0 - 0.01) * Box(10, a.clamp_slit, 30, align=(Align.MIN, Align.CENTER, Align.MIN))
    y_head, y_nut = cy - a.clamp_slit / 2 - t1, cy + a.clamp_slit / 2 + t2
    body -= Pos(cx + ear_x, y_head, ear_z) * Rot(90, 0, 0) * drive.countersunk(d, depth=10, up=5)
    body -= Pos(cx + ear_x, y_nut + 0.01, ear_z) * Rot(90, 0, 0) * Rot(0, 0, 30) * _nut(a.nut_af, a.nut_t + 0.01)
    # re-open the motor bore (the ears and arms must not reach into it)
    body -= Pos(cx, cy, h["motor"]) * Cylinder((p.motor.d + f.motor_fit) / 2, 30, align=MIN)
    body.label, body.color = "fan_yoke", (0.9, 0.55, 0.2)
    return body


def basket(p: Params = P, a: AltParams = A, fr=frame.FR, d=drive.D):
    """The battery box's floor and posts, low side walls, full front and rear walls with strap windows."""
    z1 = d.tray_bottom_z + fr.floor_t
    zt = frame.box_top(p, fr, d)
    x0, x1, y0, y1 = frame.tray_box(p, fr)
    cx = (x0 + x1) / 2
    body = frame.floor(p, fr, d)
    outer = RectangleRounded(x1 - x0, y1 - y0, fr.corner_r)
    inner = RectangleRounded(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t, fr.corner_r - fr.wall_t)
    walls = Pos(cx, 0, z1) * extrude(outer - inner, zt + a.rim - z1)
    # the long cells' ends only need locating: the side walls stay low (the front and rear walls carry the straps)
    for s in (1, -1):
        walls -= Pos(cx, s * (y1 - fr.wall_t / 2), z1 + a.side_wall_h) * Box(
            x1 - x0 - 2 * fr.corner_r, 3 * fr.wall_t, 40, align=MIN)
    body += walls
    # strap windows in the front and rear walls, just above the pack top (the strap presses on the cells)
    for s in (1, -1):
        body -= Pos(cx, s * a.strap_y, zt - 0.3) * Box(x1 - x0 + 2, a.strap_w + 0.6, a.strap_t + 0.4, align=MIN)
    sw, sh = fr.wire_slot  # battery wires out at the rear
    for y in (y0, y1):
        body -= Pos(x0 + sw / 2 + fr.wall_t + 1, y, z1) * Box(sw, 3 * fr.wall_t, sh, align=MIN)
    body.label, body.color = "basket", (0.25, 0.25, 0.28)
    return body


def straps(p: Params = P, a: AltParams = A, fr=frame.FR, d=drive.D):
    """The two velcro straps (for the picture): over the pack, through the windows, folded back on top."""
    zt = frame.box_top(p, fr, d)
    x0, x1, _, _ = frame.tray_box(p, fr)
    out = {}
    for i, s in enumerate((1, -1)):
        band = Pos((x0 + x1) / 2, s * a.strap_y, zt - 0.1) * Box(x1 - x0 + 2.4, a.strap_w, a.strap_t, align=MIN)
        band += Pos((x0 + x1) / 2 + 1, s * a.strap_y, zt + a.strap_t - 0.1) * Box(x1 - x0 - 2, a.strap_w, a.strap_t, align=MIN)
        band.label, band.color = f"velcro_{i}", (0.1, 0.1, 0.1)
        out[band.label] = band
    return out


def parts(p: Params = P):
    """Printed parts of the alternative (with the straps for the picture)."""
    printed = {**drive.all_parts(p), **front.parts(p)}
    for side in (1, -1):
        cap = block_cap(side, p)
        printed[cap.label] = cap
    printed["impeller"] = fan.impeller(p)
    printed["fan_yoke"] = fan_yoke(p)
    printed["basket"] = basket(p)
    printed.update(straps(p))
    return printed
