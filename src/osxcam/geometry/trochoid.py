"""Trochoidal looping math.

Given a *guide curve* (the centre line a band of material is cleared along),
emit a chain of overlapping circular loops. Each loop is a full circle of the
tool CENTRE about a point on the guide; consecutive loop centres advance by
``pitch`` along the guide and are joined by a short linear move.

Engagement model
----------------
With loop radius ``r`` and centre advance ``pitch`` per revolution, the strip
of fresh material cut on the leading edge of each loop has radial width ~=
``pitch`` as long as loops overlap (``pitch < 2r``). Setting ``pitch`` equal to
the desired radial engagement (``stepover``) is therefore what keeps the tool
engagement angle bounded -- the defining property of trochoidal/adaptive
clearing. This is a pragmatic simplification of true constant-engagement
clearing, not an exact constant-angle solution.

A complete loop is encoded as two semicircle arcs (start -> diametrically
opposite point -> start) so every arc move has distinct endpoints, which keeps
the G-code valid (G2/G3 with identical start==end is ambiguous on many
controllers).
"""

from __future__ import annotations

from .primitives import Point, add, normalize, perpendicular, scale, sub
from ..cam.toolpath import Toolpath


def _stations(coords: list[Point], pitch: float) -> list[Point]:
    """Resample a (closed) polyline into points spaced ~``pitch`` apart."""
    if len(coords) < 2:
        return list(coords)

    # cumulative arc length
    cum = [0.0]
    for a, b in zip(coords, coords[1:]):
        cum.append(cum[-1] + (sub(b, a)[0] ** 2 + sub(b, a)[1] ** 2) ** 0.5)
    total = cum[-1]
    if total < 1e-9:
        return [coords[0]]

    n = max(1, int(round(total / pitch)))
    step = total / n

    out: list[Point] = []
    seg = 0
    for i in range(n):
        d = i * step
        while seg < len(cum) - 2 and cum[seg + 1] < d:
            seg += 1
        seg_len = cum[seg + 1] - cum[seg]
        t = 0.0 if seg_len < 1e-12 else (d - cum[seg]) / seg_len
        a, b = coords[seg], coords[seg + 1]
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def trochoidal_path(guide: list[Point], loop_radius: float, pitch: float,
                    ccw: bool = True) -> Toolpath | None:
    """Build a chain of overlapping loops along a closed ``guide`` ring.

    Returns ``None`` if the guide is degenerate.
    """
    if loop_radius <= 1e-6 or pitch <= 1e-6:
        return None

    centers = _stations(guide, pitch)
    if len(centers) < 2:
        return None

    # Start each loop offset to one side of the guide so the short connecting
    # move between loops runs roughly tangent to the guide (length ~= pitch).
    def offset_dir(i: int) -> Point:
        nxt = centers[(i + 1) % len(centers)]
        return normalize(perpendicular(sub(nxt, centers[i])))

    first_start = add(centers[0], scale(offset_dir(0), loop_radius))
    tp = Toolpath(start=first_start)

    n = len(centers)
    for i in range(n):
        c = centers[i]
        start = add(c, scale(offset_dir(i), loop_radius))
        opposite = sub(scale(c, 2.0), start)  # diametrically opposite point

        if i > 0:
            tp.linear(start)            # advance to this loop's entry
        tp.arc(opposite, c, ccw=ccw)    # first semicircle
        tp.arc(start, c, ccw=ccw)       # second semicircle -> full loop

    return tp
