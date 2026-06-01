"""Small vector / angle helpers and circular-arc sampling.

Heavy lifting (offsets, booleans, containment) is left to Shapely; this module
only holds the bits Shapely does not give us directly.
"""

from __future__ import annotations

import math

Point = tuple[float, float]
TAU = 2.0 * math.pi


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def scale(a: Point, k: float) -> Point:
    return (a[0] * k, a[1] * k)


def length(a: Point) -> float:
    return math.hypot(a[0], a[1])


def normalize(a: Point) -> Point:
    n = length(a)
    if n < 1e-12:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def perpendicular(a: Point) -> Point:
    """Left normal (90 deg CCW) of a vector."""
    return (-a[1], a[0])


def angle_of(center: Point, p: Point) -> float:
    return math.atan2(p[1] - center[1], p[0] - center[0])


def sample_arc(start: Point, end: Point, center: Point, ccw: bool,
               max_step_deg: float = 6.0) -> list[Point]:
    """Polyline approximation of a circular arc, for the visualizer.

    If ``start == end`` the arc is treated as a full circle (sweep = 2*pi),
    matching how the trochoid generator encodes a complete loop.
    """
    r = length(sub(start, center))
    a0 = angle_of(center, start)
    a1 = angle_of(center, end)

    if ccw:
        sweep = a1 - a0
        while sweep <= 1e-9:
            sweep += TAU
    else:
        sweep = a1 - a0
        while sweep >= -1e-9:
            sweep -= TAU

    steps = max(2, int(math.ceil(abs(sweep) / math.radians(max_step_deg))))
    pts: list[Point] = []
    for i in range(1, steps + 1):
        a = a0 + sweep * (i / steps)
        pts.append((center[0] + r * math.cos(a), center[1] + r * math.sin(a)))
    return pts
