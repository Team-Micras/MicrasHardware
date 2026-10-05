"""The bought fasteners in place: M2x5 countersunk screws, M2 nuts and M2x2 glued inserts (README "Fasteners").

Each one is placed where the part that holds it cuts its hole, from the same helpers (drive.board_holes,
drive.cap_screws, drive.ring_clamps, drive.frame_boss, the fan mount's tabs and clamp), so they follow the layout.
HOSTS names the parts each one sits in: check_layout lets them touch those, but not overlap them.
"""

from build123d import Cone, Cylinder, Plane, Pos, RegularPolygon, Rot, extrude, mirror

from . import drive, fan
from .drive import D
from .params import P, Params

STEEL, ZINC, BRASS = (0.62, 0.63, 0.67), (0.5, 0.52, 0.56), (0.82, 0.66, 0.3)
MASS = {"screw": 0.13, "nut": 0.07, "insert": 0.08}  # g, steel M2x5 flat head, steel M2 nut, brass M2x2 insert


def screw(d=D):
    """M2 x screw_l countersunk: the head's top face at z = 0, the shank towards -z (the length includes the head)."""
    h = (d.csk_d - d.screw_clear_d) / 2
    return Pos(0, 0, -h) * Cone(1.0, 1.9, h, align=drive.MIN) + Pos(0, 0, -h + 0.01) * Cylinder(
        1.0, d.screw_l - h + 0.01, align=drive.MAX)


def nut(d=D):
    """M2 nut, 4.0 across flats, its hole on z, from z = 0 up (oriented like drive.nut_trap)."""
    return extrude(RegularPolygon(4.0 / 3 ** 0.5, 6), d.nut_t) - Cylinder(1.0, 10)


def insert(d=D):
    """M2 x insert_l, OD 3.2, its top at z = 0."""
    return Cylinder(1.6, d.insert_l, align=drive.MAX) - Cylinder(1.0, 10)


def _blocks(side, p):
    tag = "L" if side > 0 else "R"
    return (f"block_{tag}",) if drive.solid(p) else (f"block_base_{tag}", f"block_cap_{tag}")


def _placed(p: Params = P, d=D):
    """[(label, shape, hosts)]."""
    out = []
    az = p.axle_z
    for side in (1, -1):
        tag = "L" if side > 0 else "R"
        blocks = _blocks(side, p)
        base, cap = blocks[0], blocks[-1]
        left = []  # built in the left frame, mirrored for the right side
        # board screws from under the board (heads in the board's countersinks) into nuts slid into the base
        for i, (hx, hy, a) in enumerate(drive.board_holes(p, d), 1):
            left.append((f"screw_board_{tag}{i}", Pos(hx, hy, p.board.bottom_z) * Rot(180, 0, 0) * screw(d),
                         (base, f"nut_board_{tag}{i}")))
            left.append((f"nut_board_{tag}{i}", Pos(hx, hy, p.board.top_z + d.board_nut_z) * Rot(0, 0, a) * nut(d),
                         (base, f"screw_board_{tag}{i}")))
        # cap screws into the inserts glued in the base (split blocks)
        if not drive.solid(p):
            for i, (sx, sy) in enumerate(drive.cap_screws(side, p, d), 1):
                left.append((f"screw_cap_{tag}{i}", Pos(sx, sy, az + d.screw_l - d.insert_l) * screw(d),
                             (base, cap, f"insert_cap_{tag}{i}")))
                left.append((f"insert_cap_{tag}{i}", Pos(sx, sy, az - 0.05) * insert(d),
                             (base, cap, f"screw_cap_{tag}{i}")))  # (just under the cap's relieved face)
        # motor ring clamps: down through the upper ear into a nut trapped under the lower one
        for i, (a, o, y_max) in enumerate(drive.ring_clamps(side, p, d), 1):
            (ex, ey, ez), (_, _, ear_h) = drive.ring_clamp_screw(a, o, y_max, p, d)
            left.append((f"screw_clamp_{tag}{i}", Pos(ex, ey, ez + ear_h / 2) * screw(d), (cap, f"nut_clamp_{tag}{i}")))
            left.append((f"nut_clamp_{tag}{i}", Pos(ex, ey, ez - ear_h / 2) * Rot(0, 0, 30) * nut(d),
                         (cap, f"screw_clamp_{tag}{i}")))
        # the basket's insert in the cap's frame boss
        fx, fy, ftop = drive.frame_boss(side, p, d)
        left.append((f"insert_frame_{tag}", Pos(fx, fy, ftop) * insert(d), (cap, f"screw_basket_{tag}")))
        # the fan mount's tab onto the cap's ear: a nut trapped under the ear
        tx, ty = d.fan_ear
        zt = drive.fan_ear_top(p, d)
        left.append((f"nut_fan_{tag}", Pos(tx, ty, zt - d.fan_ear_t) * nut(d), (cap, f"screw_fan_{tag}")))
        left.append((f"screw_fan_{tag}", Pos(tx, ty, zt - d.fan_recess + fan.F.tab_t) * screw(d),
                     (cap, "fan_mount", f"nut_fan_{tag}")))
        # the basket's screw: through its floor (on the left, at the foot of its post) into the frame boss's insert
        from .frame import FR
        floor = d.tray_bottom_z + FR.floor_t
        left.append((f"screw_basket_{tag}", Pos(fx, fy, ftop + FR.floor_t if ftop < d.tray_bottom_z else floor)
                     * screw(d), (cap, "basket", f"insert_frame_{tag}", "cell0", "cell1", "cell2")))  # (flush under the cells)
        for label, shape, hosts in left:
            out.append((label, mirror(shape, Plane.XZ) if side < 0 else shape, hosts))
    # the fan motor's clamp: across the collar's split, head in one ear, the nut trapped in the other
    f = fan.F
    cx, cy = fan.centre(p)
    ex, ez = fan.clamp_screw(p, f, d)
    t1, t2 = f.clamp_ear_t
    out.append(("screw_fan_clamp", Pos(cx + ex, cy - f.clamp_slit / 2 - t1, ez) * Rot(90, 0, 0) * screw(d),
                ("fan_mount", "nut_fan_clamp")))
    out.append(("nut_fan_clamp", Pos(cx + ex, cy + f.clamp_slit / 2 + t2, ez) * Rot(90, 0, 0) * nut(d),
                ("fan_mount", "screw_fan_clamp")))
    return out


def parts(p: Params = P, d=D):
    out = {}
    for label, shape, _ in _placed(p, d):
        kind = label.split("_")[0]
        shape.label, shape.color = label, {"screw": STEEL, "nut": ZINC, "insert": BRASS}[kind]
        out[label] = shape
    return out


def hosts(p: Params = P, d=D):
    """{fastener: (the parts it sits in)}."""
    return {label: h for label, _, h in _placed(p, d)}
