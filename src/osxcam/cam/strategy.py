"""Selectable cut strategies (machining modes).

osxCAM started with a single strategy -- trochoidal adaptive pocket clearing.
This module adds a small enum of *cut modes* and a single dispatcher so the UI
and G-code stages can ask for a strategy by name:

- **Pocket (adaptive)**  -- the existing trochoidal area-clearing path.
- **Profile -- outside**  -- a contour offset *outward* by the tool radius, so
  the cutter rides the part's outer wall (cut a part free from stock).
- **Profile -- inside**   -- a contour offset *inward* by the tool radius, so
  the wall is left at the nominal size (cut a hole/window to size).
- **Engrave (on line)**   -- follow the selected profile centreline with no
  offset (v-bit / score lines).

The contour modes clear no area -- they trace boundaries -- so they reuse the
Z-layer / plunge / retract wrapping the G-code stage already applies to every
:class:`Toolpath`.
"""

from __future__ import annotations

from enum import Enum

from shapely.geometry import MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry

from .adaptive import clear_pocket
from .params import JobParams, ToolParams
from .toolpath import Toolpath
from ..geometry.region import JOIN_ROUND
from ..geometry.primitives import Point


class CutMode(Enum):
    """Machining strategy. ``value`` doubles as the UI label."""

    POCKET = "Pocket (adaptive)"
    PROFILE_OUTSIDE = "Profile — outside"
    PROFILE_INSIDE = "Profile — inside"
    ENGRAVE = "Engrave (on line)"

    @property
    def is_contour(self) -> bool:
        return self is not CutMode.POCKET


def _offset_geometry(pocket: Polygon, mode: CutMode,
                     tool_radius: float) -> BaseGeometry:
    """Boundary the cutter CENTRE follows for a contour mode.

    Outside grows the filled region by the tool radius, inside erodes it, and
    engrave follows the profile as-drawn. A round join keeps corners tool-safe.
    """
    if mode is CutMode.PROFILE_OUTSIDE:
        return pocket.buffer(tool_radius, join_style=JOIN_ROUND)
    if mode is CutMode.PROFILE_INSIDE:
        return pocket.buffer(-tool_radius, join_style=JOIN_ROUND)
    return pocket  # ENGRAVE: on the line, no offset


def _polygon_rings(poly: Polygon):
    yield list(poly.exterior.coords), False
    for interior in poly.interiors:
        yield list(interior.coords), True


def _all_rings(geom: BaseGeometry):
    """Yield ``(coords, is_hole)`` for every ring of a Polygon/MultiPolygon."""
    if geom.is_empty:
        return
    if isinstance(geom, Polygon):
        yield from _polygon_rings(geom)
    elif isinstance(geom, MultiPolygon):
        for g in geom.geoms:
            if not g.is_empty:
                yield from _polygon_rings(g)


def _ring_path(coords: list[Point], ccw: bool) -> Toolpath:
    """A closed contour as straight moves, traversed CW or CCW.

    ``coords`` is a closed ring (first == last). Shapely's signed area tells us
    the current winding; reverse it if the requested direction differs.
    """
    if len(coords) < 4:
        return Toolpath(coords[0] if coords else (0.0, 0.0))
    # signed area (shoelace); >0 == CCW
    area2 = sum(coords[i][0] * coords[i + 1][1] - coords[i + 1][0] * coords[i][1]
                for i in range(len(coords) - 1))
    if (area2 > 0) != ccw:
        coords = coords[::-1]
    tp = Toolpath((coords[0][0], coords[0][1]))
    for x, y in coords[1:]:
        tp.linear((x, y))
    return tp


def _contour_paths(pocket: Polygon, tool: ToolParams, job: JobParams,
                   mode: CutMode) -> list[Toolpath]:
    geom = _offset_geometry(pocket, mode, tool.radius_mm)
    if geom.is_empty:
        raise ValueError(
            "tool too large for this profile (offset contour is empty)")
    paths: list[Toolpath] = []
    for coords, is_hole in _all_rings(geom):
        # Climb milling: outer walls run CCW, holes/islands run CW (and the
        # reverse for conventional). A starting point -- verify by chip & sound.
        ccw = job.climb != is_hole
        tp = _ring_path(coords, ccw)
        if tp.moves:
            paths.append(tp)
    if not paths:
        raise ValueError("no contour generated for this profile")
    return paths


def make_paths(pocket: Polygon, tool: ToolParams, job: JobParams,
               mode: CutMode) -> list[Toolpath]:
    """Toolpaths for one shape under ``mode``. Raises ``ValueError`` if the
    tool cannot fit (empty machinable region / empty offset)."""
    if mode is CutMode.POCKET:
        return clear_pocket(pocket, tool, job)
    return _contour_paths(pocket, tool, job, mode)


def make_paths_multi(pockets: list[Polygon], tool: ToolParams, job: JobParams,
                     mode: CutMode) -> tuple[list[Toolpath], list[int]]:
    """Run ``mode`` over several shapes into one path list.

    Shapes the tool cannot handle are skipped (their indices returned) rather
    than aborting the batch -- matches ``clear_pockets`` for thin letters.
    """
    all_paths: list[Toolpath] = []
    skipped: list[int] = []
    for i, pocket in enumerate(pockets):
        try:
            all_paths.extend(make_paths(pocket, tool, job, mode))
        except ValueError:
            skipped.append(i)
    return all_paths, skipped
