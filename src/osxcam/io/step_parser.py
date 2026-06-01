"""STEP -> horizontal planar faces -> 2D pocket Polygons.

Uses pythonocc-core (OpenCASCADE). That library is distributed via conda-forge,
not PyPI:

    conda install -c conda-forge pythonocc-core

The OCC imports are therefore done lazily inside the functions so the rest of
the app (SVG/DXF import, UI) keeps working in a plain pip venv without OCC.

Strategy
--------
1. Read the STEP shape.
2. Walk every face; keep the PLANAR ones whose normal is parallel to Z
   (horizontal floors/tops) within ``angle_tol_deg``.
3. For each kept face, discretise its outer wire (boundary) and inner wires
   (holes / islands) into ordered XY polylines -- Z is dropped since the face
   is horizontal -- and build a Shapely ``Polygon(outer, holes=...)``.
4. Return faces sorted top-down (highest Z first), then by area, so the UI's
   Shape dropdown lists the most likely pocket floor first.

The resulting polygons feed the existing trochoidal engine unchanged.
"""

from __future__ import annotations

import math

from shapely.geometry import Polygon

from ..geometry.primitives import Point

PYOCC_HINT = ("STEP import requires pythonocc-core (OpenCASCADE).\n"
              "Install it with:  conda install -c conda-forge pythonocc-core")


def _require_occ():
    try:
        import OCC.Core  # noqa: F401
    except ImportError as exc:  # pragma: no cover - depends on env
        raise ImportError(PYOCC_HINT) from exc


def _discretize_wire(wire, deflection: float, scale: float) -> list[Point]:
    """Ordered XY points around a wire, honouring edge orientation."""
    from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
    from OCC.Core.BRepTools import BRepTools_WireExplorer
    from OCC.Core.GCPnts import GCPnts_QuasiUniformDeflection
    from OCC.Core.TopAbs import TopAbs_REVERSED

    pts: list[Point] = []
    wexp = BRepTools_WireExplorer(wire)
    while wexp.More():
        edge = wexp.Current()
        curve = BRepAdaptor_Curve(edge)
        sampler = GCPnts_QuasiUniformDeflection(curve, deflection)
        epts: list[Point] = []
        if sampler.IsDone():
            for i in range(1, sampler.NbPoints() + 1):
                p = sampler.Value(i)
                epts.append((p.X() * scale, p.Y() * scale))
        if edge.Orientation() == TopAbs_REVERSED:
            epts.reverse()
        # avoid duplicating the shared vertex between consecutive edges
        if pts and epts and _close(pts[-1], epts[0]):
            epts = epts[1:]
        pts.extend(epts)
        wexp.Next()
    return pts


def _close(a: Point, b: Point, tol: float = 1e-7) -> bool:
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


def parse_step(filepath: str, scale: float | None = None,
               angle_tol_deg: float = 1.0,
               deflection: float = 0.05) -> list[Polygon]:
    _require_occ()

    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
    from OCC.Core.BRepTools import breptools
    from OCC.Core.GeomAbs import GeomAbs_Plane
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.Interface import Interface_Static
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_WIRE
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopoDS import topods

    factor = 1.0 if scale is None else scale
    Interface_Static.SetCVal("xstep.cascade.unit", "MM")

    reader = STEPControl_Reader()
    if reader.ReadFile(filepath) != IFSelect_RetDone:
        raise ValueError(f"could not read STEP file: {filepath}")
    reader.TransferRoots()
    shape = reader.OneShape()

    cos_tol = math.cos(math.radians(angle_tol_deg))
    candidates: list[tuple[float, float, Polygon]] = []  # (z, area, polygon)

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = topods.Face(exp.Current())
        exp.Next()

        surf = BRepAdaptor_Surface(face, True)
        if surf.GetType() != GeomAbs_Plane:
            continue
        pln = surf.Plane()
        normal = pln.Axis().Direction()
        if abs(normal.Z()) < cos_tol:   # not horizontal
            continue
        z = pln.Location().Z() * factor

        outer = breptools.OuterWire(face)
        outer_pts = _discretize_wire(outer, deflection, factor)
        if len(outer_pts) < 3:
            continue

        holes: list[list[Point]] = []
        wexp = TopExp_Explorer(face, TopAbs_WIRE)
        while wexp.More():
            wire = topods.Wire(wexp.Current())
            wexp.Next()
            if wire.IsSame(outer):
                continue
            hpts = _discretize_wire(wire, deflection, factor)
            if len(hpts) >= 3:
                holes.append(hpts)

        poly = Polygon(outer_pts, holes=holes)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or poly.area < 1e-9:
            continue
        candidates.append((z, poly.area, poly))

    if not candidates:
        raise ValueError("no horizontal planar faces found in STEP file")

    candidates.sort(key=lambda c: (-c[0], -c[1]))  # top-down, then largest
    return [c[2] for c in candidates]
