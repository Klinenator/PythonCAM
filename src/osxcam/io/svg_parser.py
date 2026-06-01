"""SVG -> list of closed coordinate rings (millimetres, Y-up).

Uses svgelements with ``reify=True`` so element/group transforms are baked into
the coordinates. SVG is Y-down; we flip Y about the document bounding box so the
output matches CAM convention (Y-up).

Unit handling: SVG user units are pixels by default. ``units`` selects the
assumed real-world meaning of one user unit:
    'px' -> 96 dpi  (1 px = 25.4/96 mm)   [default]
    'mm' -> 1:1
    'in' -> 25.4 mm per unit
A direct ``scale`` (mm per user unit) overrides ``units`` when given.
"""

from __future__ import annotations

from svgelements import SVG, Path, Shape

from ..geometry.primitives import Point

_UNIT_MM = {"px": 25.4 / 96.0, "mm": 1.0, "in": 25.4}
_FLATNESS = 0.05  # mm-ish error tolerance for sampling curves


def _sample_subpath(subpath, n_min: int = 16) -> list[Point]:
    length = subpath.length(error=1e-3)
    n = max(n_min, int(length / max(_FLATNESS, 1e-6)))
    n = min(n, 4000)
    pts = [(p.x, p.y) for p in (subpath.point(i / n) for i in range(n + 1))]
    return pts


def parse_svg(filepath: str, units: str = "px",
              scale: float | None = None) -> list[list[Point]]:
    factor = scale if scale is not None else _UNIT_MM.get(units, _UNIT_MM["px"])

    svg = SVG.parse(filepath, reify=True)

    rings: list[list[Point]] = []
    for element in svg.elements():
        if not isinstance(element, (Path, Shape)):
            continue
        try:
            path = abs(Path(element))  # resolve to absolute Path
        except Exception:
            continue
        for sub in path.as_subpaths():
            sp = Path(sub)
            pts = _sample_subpath(sp)
            if len(pts) >= 4:
                rings.append(pts)

    if not rings:
        raise ValueError("no closed paths found in SVG")

    # flip Y about document bounds, then scale to mm
    ys = [y for ring in rings for (_, y) in ring]
    ymax = max(ys)
    return [[((x * factor), ((ymax - y) * factor)) for (x, y) in ring]
            for ring in rings]
