"""DXF -> list of closed coordinate rings (millimetres, Y-up).

DXF is already Y-up and in real units; ``$INSUNITS`` tells us which. We convert
to millimetres. v1 handles the entities that actually carry closed profiles:
closed LWPOLYLINE/POLYLINE, CIRCLE, ELLIPSE, and closed SPLINE (flattened).
Open LINE/ARC fragments are not stitched into loops yet.
"""

from __future__ import annotations

import ezdxf

from ..geometry.primitives import Point

# ezdxf $INSUNITS code -> millimetres per drawing unit (common subset)
_INSUNITS_MM = {0: 1.0, 1: 25.4, 2: 304.8, 4: 1.0, 5: 10.0, 6: 1000.0}
_FLATTEN = 0.05  # mm sagitta tolerance for arcs/splines


def _close(pts: list[Point]) -> list[Point]:
    if len(pts) >= 2 and pts[0] != pts[-1]:
        pts = pts + [pts[0]]
    return pts


def parse_dxf(filepath: str, scale: float | None = None) -> list[list[Point]]:
    doc = ezdxf.readfile(filepath)
    factor = scale if scale is not None else _INSUNITS_MM.get(doc.units, 1.0)
    msp = doc.modelspace()

    rings: list[list[Point]] = []
    for e in msp:
        dxftype = e.dxftype()
        pts: list[Point] = []

        if dxftype == "LWPOLYLINE" and e.closed:
            pts = [(p[0], p[1]) for p in e.get_points("xy")]
        elif dxftype == "POLYLINE" and e.is_closed:
            pts = [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]
        elif dxftype in ("CIRCLE", "ELLIPSE"):
            pts = [(p.x, p.y) for p in e.flattening(_FLATTEN)]
        elif dxftype == "SPLINE" and e.closed:
            pts = [(p.x, p.y) for p in e.flattening(_FLATTEN)]
        else:
            continue

        pts = _close(pts)
        if len(pts) >= 4:
            rings.append([(x * factor, y * factor) for (x, y) in pts])

    if not rings:
        raise ValueError("no closed profiles found in DXF")
    return rings
