"""Skids: two PTFE mouse skates stuck under the board on its centre line, one behind the nose, one ahead of the tail.

The robot stands on one axle with its centre of mass over it, so the body tips forward and back about the axle
until something touches the floor, every time the acceleration changes sign. The fan's suction acts on the whole
skirted area, centred about 7 mm ahead of the axle, so with the fan on (4 N) the nose rests on the front skate
(about 0.65 N on it) unless the robot accelerates harder than about 20 m/s^2. The skates keep the rock small: they
are the lowest points ahead of and behind the axle, just clear of the floor with the tires pressed down by the
weight and the fan, so the body tips by a fraction of a degree and lands on PTFE instead of on the board's edge.

The skates are cut from a 0.7 mm sheet of mouse feet (PTFE with its adhesive): their faces sit 0.5 mm above the
floor with the tires unloaded (the board rides 1.2 up), and the tires sink about 0.35-0.5 mm under the weight and the
fan's 4 N (Shore 20 rubber, docs/fan_study.md), which leaves 0-0.15 mm of running clearance (README, assembly: the
check).
"""

from dataclasses import dataclass

from build123d import Ellipse, Pos, extrude, fillet

from .params import P, Params


@dataclass(frozen=True)
class SkidParams:
    thickness: float = 0.7  # the skate sheet, adhesive included
    # cut as an oval, the long axis along x, its edge rounded with fine sandpaper so floor seams don't catch it
    length: float = 10.0
    width: float = 7.0
    edge_r: float = 0.3
    # centres (x, y), each just inside the skirt's taped band (3 mm in from the board edge: the margin that
    # seals the gap hangs from it, so the skates stay off it): the front one behind the band at the nose (board edge
    # x 53.5), between the diagonal sensors' legs; the rear one ahead of the band at the USB notch (board edge
    # x -34.1), between the USB shell's through-hole tabs (y 2.6 and -6.1, at x -29.1 and -24.9)
    front: tuple = (45.3, 0.0)
    rear: tuple = (-25.9, -1.75)


SK = SkidParams()


def contact_z(p: Params = P, sk: SkidParams = SK):
    """Height of the skates' faces above the floor, with the tires unloaded."""
    return p.board.bottom_z - sk.thickness


def skate(p: Params = P, sk: SkidParams = SK):
    """One skate at x = y = 0 in the robot frame, stuck to the board's underside."""
    body = Pos(0, 0, contact_z(p, sk)) * extrude(Ellipse(sk.length / 2, sk.width / 2), sk.thickness)
    return fillet(body.edges().group_by()[0], sk.edge_r)


def centres(sk: SkidParams = SK):
    return {"skid_front": sk.front, "skid_rear": sk.rear}


def parts(p: Params = P, sk: SkidParams = SK):
    """The two skates in place."""
    out = {}
    for name, (x, y) in centres(sk).items():
        part = Pos(x, y, 0) * skate(p, sk)
        part.label, part.color = name, (0.95, 0.95, 0.95)
        out[name] = part
    return out
