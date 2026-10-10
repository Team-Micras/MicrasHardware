"""Reference layout: the board and every bought part placed in the robot frame."""

import json
from functools import lru_cache
from pathlib import Path

from build123d import Box, Compound, Pos, Rot, import_step

from . import purchased as pp
from .params import P, Params

REF = Path(__file__).resolve().parents[1] / "ref"


@lru_cache
def board():
    """Populated board from ref/board.step (tools/export_board.py), in the robot frame."""
    raw = import_step(REF / "board.step")
    # kicad-cli --grid-origin: KiCad (x, y) = (X + 84.3403, 180.6148 - Y); STEP Z=0 is the board bottom.
    to_robot = Pos(-67.1112, 64.1608, P.board.bottom_z) * Rot(0, 0, -90)
    children = []
    for c in list(raw.children):
        c.parent = None  # moving deep-copies the shape, and with a parent it would copy the whole board each time
        moved = to_robot * c
        moved.label = c.label
        children.append(moved)
    placed = Compound(children=children, label="board")
    return placed


def _svg_arc(p0, rx, large, sweep, p1, n=12):
    """Points along an SVG arc (circular: rx = ry, no rotation) from p0 to p1, p0 excluded."""
    from math import atan2, cos, pi, sin, sqrt
    (x0, y0), (x1, y1) = p0, p1
    mx, my = (x0 - x1) / 2, (y0 - y1) / 2
    r = max(rx, sqrt(mx * mx + my * my))
    h = sqrt(max(r * r - mx * mx - my * my, 0.0)) / max(sqrt(mx * mx + my * my), 1e-12)
    sign = -1 if large == sweep else 1
    cx, cy = (x0 + x1) / 2 + sign * h * my, (y0 + y1) / 2 - sign * h * mx  # (SVG spec, F.6.5)
    a0, a1 = atan2(y0 - cy, x0 - cx), atan2(y1 - cy, x1 - cx)
    da = a1 - a0
    if sweep and da < 0:
        da += 2 * pi
    elif not sweep and da > 0:
        da -= 2 * pi
    return [(cx + r * cos(a0 + da * k / n), cy + r * sin(a0 + da * k / n)) for k in range(1, n + 1)]


@lru_cache
def silkscreen(p: Params = P):
    """The board's top silkscreen (ref/board_silk.svg, tools/export_board.py) as lines on the board top, in the
    robot frame: for the viewer."""
    import re

    from build123d import Circle, Polyline

    ax, ay = json.loads((REF / "board_mech.json").read_text())["axle_kicad"]
    z = p.board.top_z + 0.01
    to_robot = lambda kx, ky: (ay - ky, ax - kx, z)  # noqa: E731
    text = (REF / "board_silk.svg").read_text()
    edges = []
    for d in re.findall(r'\sd="([^"]*)"', text, re.S):
        tokens = re.findall(r"[MLAZ]|-?\d+(?:\.\d*)?", d)
        paths, cur, i, cmd = [], [], 0, "L"
        while i < len(tokens):
            t = tokens[i]
            if t in "MLAZ":
                cmd, i = t, i + 1
                if t == "M":
                    if len(cur) > 1:
                        paths.append(cur)
                    cur = []
                elif t == "Z" and cur:
                    cur.append(cur[0])
                continue
            if cmd == "A":
                rx, _, _, large, sweep, x, y = (float(v) for v in tokens[i:i + 7])
                cur += _svg_arc(cur[-1], rx, int(large), int(sweep), (x, y))
                i += 7
            else:
                cur.append((float(t), float(tokens[i + 1])))
                i += 2
        if len(cur) > 1:
            paths.append(cur)
        for pts in paths:
            pts = [q for k, q in enumerate(pts) if k == 0 or abs(q[0] - pts[k - 1][0]) + abs(q[1] - pts[k - 1][1]) > 1e-4]
            if len(pts) > 1:
                edges += Polyline(*[to_robot(*q) for q in pts]).edges()
    for cx, cy, r in re.findall(r'<circle cx="([-\d.]+)" cy="([-\d.]+)" r="([-\d.]+)"', text):
        if float(r) > 0:
            edges += (Pos(*to_robot(float(cx), float(cy))) * Circle(float(r))).edges()
    silk = Compound(edges, label="silkscreen")  # one flat shape (as children, the viewer lags on 9000 objects)
    silk.color = (0.95, 0.95, 0.95)
    return silk


@lru_cache
def board_boxes():
    """Axis-aligned bounding box of every board component (not the bare PCB), in the robot frame.

    Cached in ref/board_boxes.json; delete it after re-exporting the board.
    """
    cache = REF / "board_boxes.json"
    if not cache.exists() or cache.stat().st_mtime < (REF / "board.step").stat().st_mtime:
        boxes = []
        for c in board().children:
            if c.label.endswith("_PCB"):
                continue
            bb = c.bounding_box()
            boxes.append({"label": c.label, "min": list(bb.min), "max": list(bb.max)})
        cache.write_text(json.dumps(boxes))
    return json.loads(cache.read_text())


def board_keepouts():
    """Board components as box solids: fast, conservative stand-ins for clash checks."""
    out = {}
    for i, b in enumerate(board_boxes()):
        size = [hi - lo for lo, hi in zip(b["min"], b["max"])]
        centre = [(hi + lo) / 2 for lo, hi in zip(b["min"], b["max"])]
        out[f"brd:{b['label']}#{i}"] = Pos(*centre) * Box(*[max(v, 1e-3) for v in size])
    return out


def outboard(side):
    """Rotation taking local +Z to robot +Y (left, side=+1) or -Y (right, side=-1)."""
    return Rot(-90 * side, 0, 0)


def drive_side(side, p: Params = P):
    """Axle, bearings, magnet, tire, motor, pinion (and the brass wheel gear) for one side."""
    s, out = side, outboard(side)
    az = p.axle_z
    parts = {}

    def at(y_outer, part, name, x=0.0, z=az):
        part = Pos(x, s * y_outer, z) * out * part
        part.label = f"{name}_{'L' if s > 0 else 'R'}"
        parts[part.label] = part

    at(p.magnet_y + p.magnet.t, pp.magnet(p.magnet), "magnet")
    at(p.bearing_outer_y, pp.bearing(p.bearing), "bearing_outer")
    at(p.bearing_inner_y + p.bearing.w, pp.bearing(p.bearing), "bearing_inner")
    # the wheel's gear: the bought brass one, drilled and screwed to the wheel (Gears.wheel_gear = "brass"), or part
    # of the printed wheel (drive.wheel)
    if p.gears.wheel_gear == "brass":
        from .drive import brass_gear
        gear = brass_gear(side, p)
        parts[gear.label] = gear
    at(p.tire_outer_y, pp.tire(p.hub_d, p.tire_w, p.wheel), "tire")
    axle_in = p.magnet_y + p.magnet.t
    at(p.wheel_outer_y - 0.2, pp.axle(p.wheel_outer_y - 0.2 - axle_in, p.wheel), "axle")  # through the end web

    mx, mz = p.motor_axis(side)
    at(p.motor_front_y, pp.motor(p.motor), "motor", mx, mz)
    at(p.pinion_y, pp.pinion(p.gears), "pinion", mx, mz)
    return parts


def encoders(p: Params = P):
    parts = {}
    for s in (1, -1):
        pcb, chip = pp.encoder_board(p.board)
        for part, name in ((pcb, "encoder_pcb"), (chip, "encoder_chip")):
            # encoder_board() puts the chip on local +Y: mirror for the right side
            placed = Pos(0, s * p.board.encoder_slot_y, p.board.top_z) * Rot(0, 0, 0 if s > 0 else 180) * part
            placed.label = f"{name}_{'L' if s > 0 else 'R'}"
            parts[placed.label] = placed
    return parts


def battery(p: Params = P):
    from .mass import battery_cells
    L, W, T = p.battery.cell
    cells, _ = battery_cells(p.battery.arrangement, p)
    out = {}
    for i, (cx, cz) in enumerate(cells):
        flat = p.battery.arrangement != "edge"
        part = pp.cell(p.battery) if flat else Rot(0, 90, 0) * pp.cell(p.battery)
        part = Pos(p.battery.x + cx, 0, p.battery.floor_z + cz) * part
        part.label, part.color = f"cell{i}", (0.2, 0.4, 0.85)
        out[part.label] = part
    return out


def reference(p: Params = P, with_board=True):
    parts = {}
    parts.update(battery(p))
    parts.update(encoders(p))
    parts.update(drive_side(1, p))
    parts.update(drive_side(-1, p))
    if with_board:
        parts["board"] = board()
    return parts


def summary(p: Params = P):
    lx, lz = p.motor_axis(1)
    rx, rz = p.motor_axis(-1)
    return {
        "axle_z": p.axle_z,
        "wheel_d": p.wheel_d,
        "hub_d": p.hub_d,
        "tire_t_stretched": p.wheel.stretched_t(p.hub_d),
        "tire_w": p.tire_w,
        "track": p.track,
        "hall_y": p.hall_y,
        "magnet_y": p.magnet_y,
        "gear_y": p.gear_y,
        "gear_floor_clearance": p.axle_z - p.gears.tip_d(p.gears.wheel_z) / 2,
        "center_distance": p.gears.center_distance,
        "motor_L_axis_xz": (lx, lz),
        "motor_R_axis_xz": (rx, rz),
        "motor_front_y": p.motor_front_y,
        "motor_rear_y": p.motor_front_y - p.motor.body_l - p.motor.rear_l,
    }


@lru_cache(maxsize=1)
def pcb_face():
    """The PCB outline (with its cut-outs and holes) as a face at z = 0, in the robot frame."""
    from build123d import Circle, Line, ThreePointArc, Wire, make_face

    mech = json.loads((REF / "board_mech.json").read_text())
    edges = []
    for g in mech["edge"]:
        if g["type"] == "line":
            if sum((u - v) ** 2 for u, v in zip(g["start"], g["end"])) > 1e-8:
                edges.append(Line(tuple(g["start"]), tuple(g["end"])))
        elif g["type"] == "arc":
            edges.append(ThreePointArc(tuple(g["start"]), tuple(g["mid"]), tuple(g["end"])))
    wires = sorted(Wire.combine([e for e in edges if e.length > 1e-3]), key=lambda w: -w.bounding_box().size.X)
    face = make_face(wires[0])
    for w in wires[1:]:
        if w.is_closed:
            face -= make_face(w)
    for h in mech["holes"]:
        face -= Pos(*h["pos"]) * Circle(h["drill"] / 2)
    return face


def board_simple(p: Params = P):
    """Light board stand-in for the viewer: real PCB outline with holes, plus one box per component."""
    from build123d import extrude

    face = pcb_face()
    pcb = Pos(0, 0, p.board.bottom_z) * extrude(face, p.board.thickness)
    pcb.label, pcb.color = "pcb", (0.2, 0.5, 0.3)
    comps = Compound(children=list(board_keepouts().values()), label="components")
    comps.color = (0.5, 0.5, 0.45)
    return pcb, comps
