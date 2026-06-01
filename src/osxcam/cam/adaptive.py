"""Trochoidal adaptive-clearing strategy.

Orchestrates: pocket -> machinable region -> nested guide rings -> a trochoidal
toolpath per ring. Returns the XY sub-paths (one per guide ring). The G-code
stage wraps these with plunges/retracts at each Z layer.
"""

from __future__ import annotations

from shapely.geometry import Polygon

from .params import JobParams, ToolParams
from .toolpath import Toolpath
from ..geometry.region import generate_guide_rings, machinable_region
from ..geometry.trochoid import trochoidal_path


def estimate_moves(pockets: list[Polygon], tool: ToolParams,
                   job: JobParams) -> int:
    """Cheap upper-ish estimate of the move count *without* generating paths.

    Cost of a trochoidal clear scales as (area) / (loop_radius * pitch): the
    nested guide rings have total length ~= area / (2*loop_radius), and one loop
    (~3 moves) is emitted every ``pitch`` of that length. Verified within a few
    percent against real runs, so the UI can warn before a pathological job
    (tiny tool vs. large part, or a wrong import scale) churns for minutes.
    """
    loop_r = job.loop_radius(tool)
    pitch = job.pitch_mm(tool)
    if loop_r <= 1e-6 or pitch <= 1e-6:
        return 0
    total = 0.0
    for pocket in pockets:
        region = machinable_region(pocket, tool.radius_mm)
        if region.is_empty:
            continue
        total += 1.5 * region.area / (loop_r * pitch)
    return int(total)


def clear_pocket(pocket: Polygon, tool: ToolParams, job: JobParams) -> list[Toolpath]:
    region = machinable_region(pocket, tool.radius_mm)
    if region.is_empty:
        raise ValueError(
            "tool too large for this pocket (machinable region is empty)"
        )

    loop_r = job.loop_radius(tool)
    pitch = job.pitch_mm(tool)
    rings = generate_guide_rings(region, loop_r)

    # Auto-fit: the first ring needs the region to survive eroding by loop_r, so
    # a pocket narrower than tool_dia + 2*loop_r yields nothing. When the user
    # left the loop radius blank (auto), shrink it until a pass fits rather than
    # silently producing an empty toolpath. An explicit loop radius is honoured.
    if not rings and job.loop_radius_mm is None:
        floor = max(0.05, tool.radius_mm * 0.05)
        while not rings and loop_r > floor:
            loop_r *= 0.5
            rings = generate_guide_rings(region, loop_r)

    paths: list[Toolpath] = []
    for ring in rings:
        tp = trochoidal_path(ring, loop_radius=loop_r, pitch=pitch, ccw=job.climb)
        if tp is not None:
            paths.append(tp)
    return paths


def clear_pockets(pockets: list[Polygon], tool: ToolParams,
                  job: JobParams) -> tuple[list[Toolpath], list[int]]:
    """Clear several pockets into one combined path list.

    Shapes the tool cannot fit (empty machinable region) are skipped rather than
    aborting the batch -- common with thin letters. Returns the combined paths
    and the indices of any skipped shapes so the caller can report them.
    """
    all_paths: list[Toolpath] = []
    skipped: list[int] = []
    for i, pocket in enumerate(pockets):
        try:
            all_paths.extend(clear_pocket(pocket, tool, job))
        except ValueError:
            skipped.append(i)
    return all_paths, skipped
