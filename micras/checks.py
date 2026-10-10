"""Geometric checks: interferences and minimum gaps between placed parts."""

from itertools import combinations

from build123d import Shape


def _bbox_overlap(a, b, margin=0.0):
    A, B = a.bounding_box(), b.bounding_box()
    return all(tuple(A.min)[i] - margin <= tuple(B.max)[i] and
               tuple(B.min)[i] - margin <= tuple(A.max)[i] for i in range(3))


def gap(a: Shape, b: Shape) -> float:
    """Minimum distance between two shapes (0 when they touch or overlap)."""
    return a.distance_to(b)


def overlap_volume(a: Shape, b: Shape) -> float:
    if not _bbox_overlap(a, b):
        return 0.0
    common = a & b
    return common.volume if common is not None else 0.0


def clashes(parts: dict, others: dict | None = None, allowed=(), margin=0.0, tol=1e-3):
    """Pairs closer than `margin` (or overlapping), within `parts` and between `parts` and `others`.

    `allowed` holds frozensets of label pairs that are meant to touch (fits).
    """
    pairs = list(combinations(parts.items(), 2))
    if others:
        pairs += [(a, b) for a in parts.items() for b in others.items()]
    found = []
    for (na, a), (nb, b) in pairs:
        if frozenset((na, nb)) in allowed or not _bbox_overlap(a, b, margin):
            continue
        d = gap(a, b)
        if d < margin - tol or d == 0.0:
            vol = overlap_volume(a, b) if d == 0.0 else 0.0
            if d > 0 or vol > tol:
                found.append((na, nb, round(d, 3), round(vol, 3)))
    return found
