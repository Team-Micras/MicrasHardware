"""Simplified models of bought parts, each in its own local frame.

Rotational parts have their axis on local +Z. Their "front" (shaft end, outboard face) is towards +Z.
"""

from build123d import Box, Cylinder, Part, Pos, Align

from .params import P

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


def _tag(part, label, color):
    part.label = label
    part.color = color
    return part


def motor(m=P.motor, label="motor"):
    """Front face at z=0, shaft towards +Z."""
    body = Cylinder(m.d / 2, m.body_l, align=MAX)
    body += Cylinder(m.boss_d / 2, m.boss_l, align=MIN)
    body += Cylinder(m.shaft_d / 2, m.shaft_l, align=MIN)
    rear = Part()
    for s in (-1, 1):
        rear += Pos(0, s * m.terminal_pitch / 2, -m.body_l) * Box(
            m.terminal_w, m.terminal_t, m.rear_l, align=MAX)
    return _tag(body + rear, label, (0.75, 0.75, 0.78))


def spur(z, width, bore, g=P.gears, label="gear"):
    """Tip-diameter cylinder with the pitch circle marked; outer face at z=0."""
    tip = Cylinder(g.tip_d(z) / 2, width, align=MAX)
    tip -= Cylinder(bore / 2, width, align=MAX)
    return _tag(tip, label, (0.85, 0.7, 0.3))


def pinion(g=P.gears):
    return spur(g.pinion_z, g.pinion_w, g.pinion_bore, g, "pinion")


def wheel_gear(g=P.gears):
    return spur(g.wheel_z, g.wheel_w, g.wheel_bore, g, "wheel_gear")


def bearing(b=P.bearing):
    ring = Cylinder(b.od / 2, b.w, align=MAX) - Cylinder(b.id / 2, b.w, align=MAX)
    return _tag(ring, "bearing", (0.6, 0.6, 0.65))


def magnet(mg=P.magnet):
    return _tag(Cylinder(mg.d / 2, mg.t, align=MAX), "magnet", (0.8, 0.2, 0.2))


def axle(length, w=P.wheel):
    return _tag(Cylinder(w.axle_d / 2, length, align=MAX), "axle", (0.5, 0.5, 0.55))


def tire(hub_d, width, w=P.wheel):
    t = w.stretched_t(hub_d)
    ring = Cylinder(hub_d / 2 + t, width, align=MAX) - Cylinder(hub_d / 2, width, align=MAX)
    return _tag(ring, "tire", (0.15, 0.15, 0.15))


def cell(b=P.battery):
    length, width, thick = b.cell
    return _tag(Box(width, length, thick), "cell", (0.2, 0.4, 0.85))


def encoder_board(bd=P.board):
    """Daughterboard in the XZ plane, chip on +Y; origin at slot centre on the board top."""
    pcb = Pos(0, 0, -bd.thickness) * Box(5.0, bd.encoder_pcb_t, bd.thickness, align=MIN)
    pcb += Box(9.6, bd.encoder_pcb_t, bd.encoder_top_h, align=MIN)
    chip = Pos(0, bd.encoder_pcb_t / 2, bd.encoder_chip_h) * Box(
        5.0, bd.package_h, 4.4, align=(Align.CENTER, Align.MIN, Align.CENTER))
    return _tag(pcb, "encoder_pcb", (0.1, 0.45, 0.2)), _tag(chip, "encoder_chip", (0.1, 0.1, 0.1))
