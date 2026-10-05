"""Mass model: centre of mass and yaw inertia of the robot.

Printed parts use their real volumes; bought parts use estimated masses until weighed (see MASSES).
Positions are in the robot frame (mm), masses in grams.
"""

from dataclasses import dataclass

import numpy as np

from .params import P, Params

DENSITY = {"resin": 1.15e-3, "petg": 1.27e-3, "pla": 1.24e-3}  # g/mm^3

# Estimates, replace with weighed values.
MASSES = {
    "board": 15.0,  # populated main board (FR4 ~9 g + parts): weigh it
    "board_com": (6.0, 0.0, 1.5),
    "motor": 7.0,  # 1020 coreless, weighed
    "pinion": 0.12,  # printed 7T x 5 mm (the wheel's gear is printed with the wheel: counted with the printed parts)
    "tire": 0.5,
    "bearing": 0.2,
    "axle": 0.32,  # 2 mm steel, 13 mm
    "magnet": 0.19,  # 4x2 NdFeB (the 6x2: 0.42)
    "encoder_board": 0.3,
    "cell": 6.0,
    "wires": 2.0,
    "velcro": 0.25,  # each strap, 10 mm x ~50 mm hook-and-loop (estimated)
    "fan_motor": 7.0,
    "impeller": 1.0,
    "fan_housing": 1.5,
    "front": 4.0,  # placeholder for early studies (placeholders=True); the real front parts are modelled
    "front_com": (45.0, 0.0, 8.0),
    "spine": 2.5,  # bridge/spine (placeholder)
    "spine_com": (5.0, 0.0, 30.0),
    "tray": 2.0,  # battery tray + lid (placeholder)
}


@dataclass
class Item:
    name: str
    mass: float
    com: tuple
    izz: float = 0.0  # about its own vertical axis through its com, g*mm^2


def box_izz(m, sx, sy):
    return m * (sx**2 + sy**2) / 12


def summarize(items):
    m = sum(i.mass for i in items)
    com = sum(np.array(i.com) * i.mass for i in items) / m
    # yaw inertia about the vertical axis through the axle midpoint (the robot's turning axis)
    izz = sum(i.izz + i.mass * (i.com[0] ** 2 + i.com[1] ** 2) for i in items)
    return {"mass": m, "com": tuple(com), "izz": izz}


def fixed_items(p: Params = P, printed=None, fan_z=None, placeholders=True, material=None):
    """Everything except the battery.

    With placeholders=False the not-yet-designed parts (front, spine, tray) are left out: pass the real
    printed parts instead, with material(name) -> material key for their density.
    """
    M = MASSES
    if fan_z is None:
        from .fan import heights
        fan_z = heights(p)["motor"] + p.motor.body_l / 2
    items = [
        Item("board", M["board"], M["board_com"], box_izz(M["board"], 90, 50)),
        Item("fan_motor", M["fan_motor"], (p.board.fan_hole_x, 0, fan_z)),

        Item("wires", M["wires"], (-10, 0, 15)),
    ]
    from .frame import box_top, straps
    items += [Item(f"velcro_{i}", M["velcro"], (p.battery.x, 0.0, box_top(p))) for i in range(len(straps(p)))]
    if placeholders:
        items += [Item("front", M["front"], M["front_com"], box_izz(M["front"], 20, 50)),
                  Item("spine", M["spine"], M["spine_com"])]
    for s in (1, -1):
        tag = "L" if s > 0 else "R"
        mx, mz = p.motor_axis(s)
        motor_cy = s * (p.motor_front_y - p.motor.body_l / 2)
        items += [
            Item(f"motor_{tag}", M["motor"], (mx, motor_cy, mz), box_izz(M["motor"], p.motor.d, p.motor.body_l)),
            Item(f"pinion_{tag}", M["pinion"], (mx, s * (p.pinion_y - 2.5), mz)),
            Item(f"tire_axle_{tag}", M["tire"] + M["axle"], (0, s * (p.gear_y + 3), p.axle_z)),
            Item(f"bearings_{tag}", 2 * M["bearing"] + M["magnet"], (0, s * 17, p.axle_z)),
            Item(f"encoder_{tag}", M["encoder_board"], (0, s * p.board.encoder_slot_y, p.board.top_z + 6)),
        ]
    for name, part in (printed or {}).items():
        c = part.center()
        rho = DENSITY[material(name)] if material else DENSITY["resin"]
        bb = part.bounding_box()
        m = part.volume * rho
        items.append(Item(name, m, (c.X, c.Y, c.Z), box_izz(m, bb.size.X, bb.size.Y) * 0.5))
    return items


# ---- battery arrangements --------------------------------------------------------------------

def battery_cells(arrangement, p: Params = P):
    """Cell centres and footprint for an arrangement, relative to the pack's bottom-centre.

    Returns (list of (x, z) cell centres, pack size (x, z)); cells run along y.
    """
    L, W, T = p.battery.cell
    g = p.battery.gap
    if arrangement == "pyramid":  # two flat, one flat on top in the middle
        cells = [(-(W + g) / 2, T / 2), ((W + g) / 2, T / 2), (0, T + g + T / 2)]
        return cells, (2 * W + g, 2 * T + g)
    if arrangement == "edge":  # three on edge, side by side
        return [((i - 1) * (T + g), W / 2) for i in range(3)], (3 * T + 2 * g, W)
    if arrangement == "flat":  # three flat, side by side
        return [((i - 1) * (W + g), T / 2) for i in range(3)], (3 * W + 2 * g, T)
    if arrangement == "stack":  # three flat, stacked
        return [(0, T / 2 + i * (T + g)) for i in range(3)], (W, 3 * T + 2 * g)
    if arrangement == "standing":  # three upright (long side vertical), side by side
        return [((i - 1) * (T + g), L / 2) for i in range(3)], (3 * T + 2 * g, L)
    raise ValueError(arrangement)


def cell_footprint(arrangement, p: Params = P):
    """(x, y) size of one cell seen from above."""
    L, W, T = p.battery.cell
    return {"edge": (T, L), "standing": (T, W)}.get(arrangement, (W, L))


def battery_items(arrangement, x, floor_z, p: Params = P):
    L, W, T = p.battery.cell
    cells, _ = battery_cells(arrangement, p)
    items = []
    fx, fy = cell_footprint(arrangement, p)
    for i, (cx, cz) in enumerate(cells):
        items.append(Item(f"cell{i}", MASSES["cell"], (x + cx, 0, floor_z + cz), box_izz(MASSES["cell"], fx, fy)))
    return items
