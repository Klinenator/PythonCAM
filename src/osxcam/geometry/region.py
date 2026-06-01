"""Pocket region geometry: build the pocket, erode for the tool, and emit the
nested guide rings the trochoid generator runs along.

Islands (e.g. the counter of an "R") are just holes in the pocket polygon.
``buffer(-tool_radius)`` erodes the outer boundary inward AND grows the holes
outward simultaneously, so island clearance is handled for free.
"""

from __future__ import annotations

from shapely.geometry import LinearRing, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry

from .primitives import Point

JOIN_ROUND = 1  # shapely buffer join_style: round


def build_pocket(exterior: list[Point], islands: list[list[Point]] | None = None) -> Polygon:
    poly = Polygon(exterior, holes=islands or [])
    if not poly.is_valid:
        poly = poly.buffer(0)  # heal self-intersections / orientation issues
    if poly.is_empty:
        raise ValueError("pocket polygon is empty after validation")
    return poly


def machinable_region(pocket: Polygon, tool_radius: float) -> BaseGeometry:
    """Region the tool CENTRE may occupy so the cutter stays inside the pocket
    and clear of islands. May be a Polygon or MultiPolygon (or empty)."""
    return pocket.buffer(-tool_radius, join_style=JOIN_ROUND)


def _rings_of(geom: BaseGeometry) -> list[LinearRing]:
    rings: list[LinearRing] = []
    polys: list[Polygon]
    if isinstance(geom, Polygon):
        polys = [geom]
    elif isinstance(geom, MultiPolygon):
        polys = list(geom.geoms)
    else:
        polys = []
    for p in polys:
        if p.is_empty:
            continue
        rings.append(p.exterior)
        rings.extend(p.interiors)
    return rings


def generate_guide_rings(region: BaseGeometry, loop_radius: float,
                         max_rings: int = 2000) -> list[list[Point]]:
    """Nested contour-parallel guide rings filling ``region``.

    Ring k is the boundary of ``region`` eroded by ``(2k+1)*loop_radius`` so each
    ring sits at the centre of a band of width ``2*loop_radius``; adjacent bands
    tile, and the tool's own radius makes neighbouring cleared strips overlap.
    """
    guides: list[list[Point]] = []
    k = 0
    while k < max_rings:
        inset = (2 * k + 1) * loop_radius
        shell = region.buffer(-inset, join_style=JOIN_ROUND)
        if shell.is_empty:
            break
        for ring in _rings_of(shell):
            coords = list(ring.coords)
            if len(coords) >= 4:  # closed ring => first == last
                guides.append(coords)
        k += 1
    return guides
