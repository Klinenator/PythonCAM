"""Top-level import: file -> list of pocket Polygons (with islands as holes).

Dispatches to the SVG/DXF parser by extension, then classifies the flat list of
closed rings into nested shapes: the outermost ring is a boundary, and any ring
strictly contained within it becomes a hole (island) of that shape. The UI lets
the user pick which returned shape to machine.
"""

from __future__ import annotations

import os

from shapely.geometry import Polygon

from .dxf_parser import parse_dxf
from .step_parser import parse_step
from .svg_parser import parse_svg
from ..geometry.primitives import Point


def _rings_to_shapes(rings: list[list[Point]]) -> list[Polygon]:
    polys = [Polygon(r).buffer(0) for r in rings]
    polys = [p for p in polys if (not p.is_empty) and p.area > 1e-9]
    polys.sort(key=lambda p: p.area, reverse=True)

    consumed: set[int] = set()
    shapes: list[Polygon] = []
    for i, outer in enumerate(polys):
        if i in consumed:
            continue
        holes: list[list[Point]] = []
        for j in range(i + 1, len(polys)):
            if j in consumed:
                continue
            inner = polys[j]
            if outer.contains(inner.representative_point()):
                holes.append(list(inner.exterior.coords))
                consumed.add(j)
        shapes.append(Polygon(list(outer.exterior.coords), holes=holes))
    return shapes


def load_file(filepath: str, *, units: str = "px",
              scale: float | None = None) -> list[Polygon]:
    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".svg":
        rings = parse_svg(filepath, units=units, scale=scale)
    elif ext == ".dxf":
        rings = parse_dxf(filepath, scale=scale)
    elif ext in (".step", ".stp"):
        # STEP already resolves boundary vs. holes per face, so it returns
        # finished Polygons rather than a flat ring list.
        return parse_step(filepath, scale=scale)
    else:
        raise ValueError(
            f"unsupported file type: {ext!r} (use .svg, .dxf, .step, .stp)")
    return _rings_to_shapes(rings)
