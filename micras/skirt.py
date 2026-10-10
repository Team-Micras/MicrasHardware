"""Suction skirt: a plastic film taped under the board, and a nose doubler of the same film.

The film is chosen by the downforce measured on the scale rig: 10 um polyethylene bag film, 40 um cellulose acetate
or a thicker polyurethane (TPU) film, all cut from this one pattern. The inner band (tape_w) is taped to the
underside along the board edge; the lip (margin) sticks out past the edge and trails outward along the floor, so the
suction under the board presses it down. It must touch the floor: a 0.1-0.3 mm gap under it costs 12-53 % of the
downforce (docs/fan_study.md). The board's edge rides 0.9-1.5 mm up (tire sink, rocking onto the skates, 0.5 mm
steps at the maze's board joints), so the 2 mm lip reaches the floor with 0.5 mm to spare. Soft films seal best and
are too limp to hold the board up; the price is that they fold under at board joints, so a second layer of the same
film (the doubler) is laminated under the skirt along the nose. A stiff film (acetate) carries part of the robot's
weight on its lip, which takes it off the tires: trim its lip at the trim line, where it just reaches the floor.

The lip runs all the way round, the wheel notches included (an open notch leaks about 50 mm^2 and caps the fan at
about 3.5 N, docs/fan_study.md). In the notch the lip reaches |y| 19.25 at the bottom face and x +-6.5 at the end
faces, under the board's underside: the tires (inner face |y| 21.3, 2 mm up at x +-6.5) stay 0.7 mm clear and the
wheel gears 1.5 mm. The skates are stuck to the board just inside the taped band (0.2 mm in from its inner edge).

Polyethylene is a low-surface-energy plastic that ordinary tape doesn't hold: tape its band with an LSE adhesive
transfer tape (3M 9472LE) or clamp it under a Kapton tape strip laid over it.

skirt_sheet.py draws both pieces at 1:1 with rulers and an ID-1 card outline, to trace the film from a screen.
"""

from dataclasses import dataclass

from build123d import Align, Box, Pos, make_face, offset

from .layout import board_simple
from .params import P, Params


@dataclass(frozen=True)
class SkirtParams:
    margin: float = 2.0  # the lip, past the board edge: reaches the floor from 0.9-1.5 mm up
    tape_w: float = 3.0  # the taped band, inside the board edge; the skates sit just inside it
    trim: float = 1.5  # trim line past the board edge, for stiff films: the lip just reaches the floor
    doubler_x: float = 32.3  # the nose doubler covers the skirt ahead of this x: the nose, its chamfers and corners


SK = SkirtParams()


def board_outline(p: Params = P):
    """The board's outer outline (notches included, holes left out) as a face at z = 0."""
    pcb, _ = board_simple(p)
    bottom = pcb.faces().sort_by().first
    face = make_face(bottom.outer_wire())
    return Pos(0, 0, -face.bounding_box().min.Z) * face


def pattern(p: Params = P, sk: SkirtParams = SK):
    """Flat skirt shape (a face in the XY plane, robot frame, seen from above), to cut 1:1."""
    board = board_outline(p)
    return offset(board, sk.margin) - offset(board, -sk.tape_w)


def doubler(p: Params = P, sk: SkirtParams = SK):
    """The nose doubler: the skirt's band and lip ahead of doubler_x, laminated under the skirt there."""
    skirt = pattern(p, sk)
    bb = skirt.bounding_box()
    length = bb.max.X - sk.doubler_x + 1
    keep = Pos(sk.doubler_x, 0, 0) * Box(length, bb.size.Y + 2, 2, align=(Align.MIN, Align.CENTER, Align.CENTER))
    return skirt & keep


def export(out_dir, p: Params = P, sk: SkirtParams = SK):
    """build/skirt.dxf|svg (the skirt alone) and the cutting sheet, build/skirt_sheet.svg|pdf|png."""
    from build123d import ExportDXF, ExportSVG, Unit

    from . import skirt_sheet

    face = pattern(p, sk)
    for exporter, name in ((ExportDXF(unit=Unit.MM), "skirt.dxf"), (ExportSVG(unit=Unit.MM), "skirt.svg")):
        exporter.add_shape(face)
        exporter.write(str(out_dir / name))
    skirt_sheet.export(out_dir, p, sk)
    return face


if __name__ == "__main__":
    from pathlib import Path

    out = Path(__file__).resolve().parents[1] / "build"
    out.mkdir(exist_ok=True)
    export(out)
