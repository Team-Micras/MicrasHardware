"""Suction skirt: a thin plastic film (0.05-0.1 mm PET or Kapton) taped under the board.

Its inner band (tape_w) is taped to the underside along the board edge; the outer margin sticks out
past the edge and is bent down to brush the floor, closing the 1 mm gap. The margin is cut back where
the wheel gears are and along the bumper, which closes the front itself.
"""

from dataclasses import dataclass

from build123d import Box, Pos, extrude, offset

from .layout import board_simple
from .params import P, Params


@dataclass(frozen=True)
class SkirtParams:
    margin: float = 2.0  # past the board edge, bent down to the floor
    tape_w: float = 3.0  # taped band under the board
    wheel_margin: float = 0.5  # along the wheel notches
    wheel_x: float = 10.0  # |x| where the gears/tires dip low


SK = SkirtParams()


def pattern(p: Params = P, sk: SkirtParams = SK):
    """Flat skirt shape (a face in the XY plane, robot frame), to cut 1:1."""
    pcb, _ = board_simple(p)
    outline = pcb.faces().sort_by().first  # bottom face of the board
    outer_wire = outline.outer_wire()
    from build123d import make_face
    board_face = make_face(outer_wire)
    outer = offset(board_face, sk.margin)
    inner = offset(board_face, -sk.tape_w)
    skirt = outer - inner
    # cut the margin back around the wheels and at the nose skid
    for sy in (1, -1):
        keep_y = p.board.notch_inner_y + sk.wheel_margin
        skirt -= Pos(0, sy * (keep_y + 20), 0) * Box(2 * sk.wheel_x, 40, 5)
    # the bumper wraps the board's nose: no margin there
    from .front import BP
    skirt -= (Pos(BP.x_cut + 50, 0, 0) * Box(100, 200, 5)) - Pos(0, 0, -2.5) * extrude(board_face, 5)
    skirt = Pos(0, 0, -skirt.bounding_box().min.Z) * skirt
    return skirt


def export(out_dir, p: Params = P, sk: SkirtParams = SK):
    from build123d import ExportDXF, ExportSVG, Unit
    face = pattern(p, sk)
    for exporter, name in ((ExportDXF(unit=Unit.MM), "skirt.dxf"), (ExportSVG(unit=Unit.MM), "skirt.svg")):
        exporter.add_shape(face)
        exporter.write(str(out_dir / name))
    return face
