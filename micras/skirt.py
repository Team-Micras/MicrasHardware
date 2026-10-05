"""Suction skirt: a thin plastic film (0.05-0.1 mm PET or Kapton) taped under the board.

Its inner band (tape_w) is taped to the underside along the board edge; the outer margin sticks out
past the edge and is bent down to brush the floor, closing the 1 mm gap, all the way round the board, the wheel
notches included (an open notch leaked about 50 mm^2 and capped the fan at about 3.5 N, docs/fan_study.md: the
gears' lowest point is 1.54 above the floor and the tires 4 mm up at the notch ends, clear of a 2 mm margin). The
skates are stuck to the board just inside the taped band.
"""

from dataclasses import dataclass

from build123d import Pos, offset

from .layout import board_simple
from .params import P, Params


@dataclass(frozen=True)
class SkirtParams:
    margin: float = 2.0  # past the board edge, bent down to the floor
    tape_w: float = 3.0  # taped band under the board


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
    skirt = Pos(0, 0, -skirt.bounding_box().min.Z) * skirt
    return skirt


def export(out_dir, p: Params = P, sk: SkirtParams = SK):
    from build123d import ExportDXF, ExportSVG, Unit
    face = pattern(p, sk)
    for exporter, name in ((ExportDXF(unit=Unit.MM), "skirt.dxf"), (ExportSVG(unit=Unit.MM), "skirt.svg")):
        exporter.add_shape(face)
        exporter.write(str(out_dir / name))
    return face
