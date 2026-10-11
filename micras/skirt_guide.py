"""Printed guides for the suction skirt (skirt.py): templates to cut it from the film, and a jig that sticks it under
the board in the right place.

Cutting. ring() is the skirt's own shape, 4 mm tall: laid on the film over a cutting mat, the scalpel runs along its
outer wall (the lip's edge, SkirtParams.margin past the board edge) and its inner wall (the band's inner edge,
tape_w inside it). The ring covers exactly the piece that is kept, so for both cuts the film on the blade's keep
side is held flat under it and only the waste is free; it is 4.6 mm wide and 4 tall, stiff in its plane, and lies
flat on the mat. A scalpel slid along a wall cuts half its blade's thickness away from the wall, on the waste side,
so the walls stand GuideParams.blade inside the lines and the cuts land on them. nose() is the ring ahead of
SkirtParams.doubler_x, for the nose doubler: its two end faces guide the cross cuts. The trimmed pair (lip at
SkirtParams.trim, for stiff films) is optional.

Placing. jig() holds the skirt under the board the way it hangs on the robot. An island the shape of the band
(the board outline, tape_w wide) carries the skirt's band; around it a trench takes
the lip, which a soft film lets hang down over the island's edge, the board's edge, where it folds; a fence round
the trench at the lip's outline (a step lower at the trim line) is the lip's edge, for laying the film. Two pins in
the board's front-left and rear-right screw holes (P.board.front_hole, rear_hole) put the board on the island
exactly: lowered onto them, its taped underside lands on the band. Inside the band a shelf, shelf_drop below the
board, carries the pins and stiffens the island; anything else under the board (the skates, the fan's neck, the
encoder boards) is over the open middle. The board's through-hole leads are trimmed flush on the underside, but
their solder joints stand proud, so each one over the island or the shelf gets a relief hole (from the board model).
The fence stays fence_h above the board's underside, under the sensor caps and the parts that overhang the board's
edge (the lowest, the caps, start 1.0 mm above it), so the jig takes the board with its caps on; the drive comes
off first (its screws use the pins' holes, and the wheels hang through the notches).
"""

from dataclasses import dataclass

from build123d import Align, Box, Cone, Cylinder, Pos, Rectangle, extrude, offset

from .gear_guide import GP as GEAR_GUIDE
from .params import P, Params
from .skirt import SK, SkirtParams, board_outline

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass(frozen=True)
class GuideParams:
    blade: float = 0.2  # half a #11 scalpel blade (0.38 thick): its edge runs this far from the wall it slides on
    ring_h: float = 4.0  # the cutting templates' height: the blade's guide
    fit: float = GEAR_GUIDE.gear_gap  # the jig's fence round the cut film's outline (a pocket round a part)
    base: float = 1.5  # the jig's plate under the trench
    trench: float = 2.5  # island top above the trench floor: a soft film's 2 mm lip hangs into it
    shelf_drop: float = 1.0  # the shelf inside the band, under the board: clears the 0.7 skates and trimmed leads
    shelf_w: float = 5.0  # the shelf's width inside the band
    fence_h: float = 0.3  # the fence above the island at the lip's outline; half of it at the trim line
    rim: float = 4.0  # the fence's width past the lip's outline
    pin_fit: float = 0.15  # diametral, the pins in the board's Ø2.4 holes (resin outsides grow about 0.05)
    pin_h: float = 0.8  # the pins' ends above the board's top face
    pin_tip: float = 0.4  # the pins' lead-in chamfer
    relief_d: float = 2.5  # round each through-hole lead's solder joint in the island and the shelf


GP = GuideParams()


def ring(lip=None, gp: GuideParams = GP, p: Params = P, sk: SkirtParams = SK):
    """The cutting template for the skirt (robot frame, seen from above), the face on the film at z = 0."""
    lip = sk.margin if lip is None else lip
    board = board_outline(p)
    face = offset(board, lip - gp.blade) - offset(board, -sk.tape_w + gp.blade)
    part = extrude(face, gp.ring_h)
    part.label, part.color = "skirt_ring" if lip == sk.margin else "skirt_ring_trim", (0.6, 0.6, 0.65)
    return part


def nose(lip=None, gp: GuideParams = GP, p: Params = P, sk: SkirtParams = SK):
    """The cutting template for the nose doubler: the ring ahead of doubler_x, its ends on the cross cuts."""
    lip = sk.margin if lip is None else lip
    full = ring(lip, gp, p, sk)
    bb = full.bounding_box()
    x0 = sk.doubler_x + gp.blade
    keep = Pos(x0, 0, 0) * Box(bb.max.X - x0 + 1, bb.size.Y + 2, 3 * gp.ring_h,
                               align=(Align.MIN, Align.CENTER, Align.CENTER))
    part = full & keep
    part.label, part.color = "skirt_nose" if lip == sk.margin else "skirt_nose_trim", (0.6, 0.6, 0.65)
    return part


def board_leads(p: Params = P):
    """(x, y) of the board's through-hole leads below its underside (the board model, layout.board())."""
    from math import dist

    from .layout import board
    out = []
    for c in board().children:
        if c.label.endswith("_PCB") or c.bounding_box().min.Z >= p.board.bottom_z - 1e-3:
            continue
        for f in c.faces():
            bb = f.bounding_box()
            if bb.size.Z < 1e-3 and bb.max.Z < p.board.bottom_z - 1e-3:
                q = ((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2)
                if all(dist(q, r) > 0.3 for r in out):
                    out.append(q)
    return out


def pins(p: Params = P):
    """The locating pins' (x, y): the board's front-left and rear-right screw holes."""
    (fx, fy), (rx, ry) = p.board.front_hole, p.board.rear_hole
    return [(fx, fy), (rx, -ry)]


def island_z(gp: GuideParams = GP):
    return gp.base + gp.trench


def jig(gp: GuideParams = GP, p: Params = P, sk: SkirtParams = SK):
    """The placing jig (robot frame, seen from above), its bottom at z = 0: the board's underside rests on the
    island's top at island_z()."""
    from math import dist
    board = board_outline(p)
    zi = island_z(gp)
    lip_line = offset(board, sk.margin + gp.fit)
    bb = offset(board, sk.margin + gp.fit + gp.rim).bounding_box()
    outer = Pos(bb.center().X, bb.center().Y, 0) * Rectangle(bb.size.X, bb.size.Y)
    band_in = offset(board, -sk.tape_w)
    window = offset(board, -sk.tape_w - gp.shelf_w)
    part = extrude(outer, gp.base)
    part += extrude(outer - offset(board, sk.trim + gp.fit), zi + gp.fence_h / 2)
    part += extrude(outer - lip_line, zi + gp.fence_h)
    part += extrude(board - band_in, zi)
    part += extrude(band_in - window, zi - gp.shelf_drop)
    part -= Pos(0, 0, -1) * extrude(window, zi + 2)
    located = pins(p)
    for x, y in board_leads(p):
        if all(dist((x, y), q) > p.board.hole_d for q in located):
            part -= Pos(x, y, -1) * Cylinder(gp.relief_d / 2, zi + 2, align=MIN)
    r = (p.board.hole_d - gp.pin_fit) / 2
    top = zi + p.board.thickness + gp.pin_h
    for x, y in located:
        z0 = zi - gp.shelf_drop - 0.1
        part += Pos(x, y, z0) * Cylinder(r, top - gp.pin_tip - z0, align=MIN)
        part += Pos(x, y, top - gp.pin_tip) * Cone(r, r - gp.pin_tip, gp.pin_tip, align=MIN)
    part.label, part.color = "skirt_jig", (0.6, 0.6, 0.65)
    return part


def placed(gp: GuideParams = GP, p: Params = P, sk: SkirtParams = SK):
    """{label: shape}: the jig under the board, its island at the board's underside (for the pictures and checks)."""
    part = Pos(0, 0, p.board.bottom_z - island_z(gp)) * jig(gp, p, sk)
    part.label, part.color = "skirt_jig", (0.6, 0.6, 0.65)
    return {"skirt_jig": part}


def parts(gp: GuideParams = GP, p: Params = P, sk: SkirtParams = SK):
    """The skirt's ring and nose templates and the jig."""
    return {part.label: part for part in (ring(None, gp, p, sk), nose(None, gp, p, sk), jig(gp, p, sk))}


def trim_parts(gp: GuideParams = GP, p: Params = P, sk: SkirtParams = SK):
    """The templates with the lip at the trim line, for stiff films (acetate)."""
    return {part.label: part for part in (ring(sk.trim, gp, p, sk), nose(sk.trim, gp, p, sk))}
