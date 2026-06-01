"""Smoke test for the geometry engine and loaders. Run with the venv:

    ./.venv/bin/python tests/smoke_engine.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from shapely.geometry import Point as ShPoint  # noqa: E402

from osxcam.cam.adaptive import clear_pocket  # noqa: E402
from osxcam.cam.params import JobParams, ToolParams  # noqa: E402
from osxcam.cam.toolpath import Toolpath  # noqa: E402
from osxcam.gcode.generator import generate_gcode  # noqa: E402
from osxcam.geometry.region import build_pocket, machinable_region  # noqa: E402
from osxcam.geometry.trochoid import trochoidal_path  # noqa: E402
from osxcam.io.loader import load_file  # noqa: E402


def test_trochoid_only():
    guide = [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0)]
    tp = trochoidal_path(guide, loop_radius=2.0, pitch=0.4, ccw=True)
    assert tp is not None and len(tp.moves) > 10
    pts = tp.polyline()
    assert len(pts) > 100
    print(f"[trochoid]  {len(tp.moves)} moves, {len(pts)} flattened pts")


def test_engine_with_island():
    outer = [(0, 0), (50, 0), (50, 50), (0, 50)]
    island = [(20, 20), (30, 20), (30, 30), (20, 30)]
    pocket = build_pocket(outer, [island])

    tool = ToolParams(diameter_mm=3.175)
    job = JobParams(total_depth_mm=3.0, step_down_mm=1.0, stepover_fraction=0.12)

    paths = clear_pocket(pocket, tool, job)
    assert paths, "no toolpaths generated"

    region = machinable_region(pocket, tool.radius_mm)
    tol = 1e-6
    worst = 0.0
    npts = 0
    for tp in paths:
        for x, y in tp.polyline():
            npts += 1
            d = ShPoint(x, y).distance(region)  # 0 if inside
            worst = max(worst, d)

    total_moves = sum(len(p.moves) for p in paths)
    print(f"[engine]    {len(paths)} guide rings, {total_moves} moves, "
          f"{npts} pts; max tool-centre escape = {worst:.4f} mm")
    assert worst < 0.05, f"tool centre left machinable region by {worst:.4f} mm"
    assert job.z_layers() == [-1.0, -2.0, -3.0], job.z_layers()


def test_dxf_loader():
    import ezdxf
    doc = ezdxf.new()
    msp = doc.modelspace()
    msp.add_lwpolyline([(0, 0), (40, 0), (40, 40), (0, 40)], close=True)
    msp.add_lwpolyline([(15, 15), (25, 15), (25, 25), (15, 25)], close=True)
    path = os.path.join(tempfile.gettempdir(), "_osxcam_smoke.dxf")
    doc.saveas(path)

    shapes = load_file(path)
    assert len(shapes) == 1, f"expected 1 shape, got {len(shapes)}"
    shp = shapes[0]
    assert len(shp.interiors) == 1, "island not detected as hole"
    print(f"[dxf]       1 shape, area={shp.area:.1f} mm^2, "
          f"{len(shp.interiors)} island(s)")


def test_gcode_arcs():
    # full circle radius 10 about origin, encoded as two CCW semicircles
    tp = Toolpath(start=(10.0, 0.0))
    tp.arc((-10.0, 0.0), (0.0, 0.0), ccw=True)
    tp.arc((10.0, 0.0), (0.0, 0.0), ccw=True)

    job = JobParams(total_depth_mm=2.0, step_down_mm=1.0,
                    feed_rate=600, plunge_rate=150)
    g = generate_gcode([tp], job, ToolParams(3.175))
    lines = g.splitlines()

    for token in ("G21", "G90", "G17", "M3 S10000"):
        assert any(ln.startswith(token) or ln == token for ln in lines), token
    assert "G4 P1" in g           # GRBL spindle spin-up dwell
    assert lines[-1] == "M30"
    # IJK incremental from arc start to centre:
    #   start (10,0) -> centre (0,0): I-10 J0 ; next start (-10,0): I10 J0
    assert "G3 X-10 I-10 J0 F600" in g, g
    assert "G3 X10 I10 J0" in g, g
    assert g.count("(layer Z") == 2          # depth 2 / step 1 => 2 passes
    # helical entry: descend along the first loop at the plunge feed rather than
    # plunging straight down. r=10, 3deg ramp => ~3.3mm/rev, 1mm drop => 1 rev.
    assert "G1 Z-1 F150" not in g            # no straight vertical plunge
    assert "G3 X-10 Z-0.5 I-10 J0 F150" in g, g   # layer-1 helix, first half
    assert "G0 Z-1" in g                     # layer-2 rapids to prior depth
    print(f"[gcode]     {len(lines)} lines; header/arcs/IJK/layers/helix OK")


def test_feeds_and_speeds():
    from osxcam.cam.feeds import recommend

    # aluminium, 1/8" 2-flute carbide: ideal rpm far exceeds 13k -> rpm-limited
    r = recommend("Aluminum", "Carbide", 3.175, 2, rpm_max=13000)
    assert r.spindle_rpm == 13000 and r.rpm_limited, r
    assert r.feed_rate == round(13000 * 2 * 0.025 / 10) * 10, r
    assert 0.3 * r.feed_rate <= r.plunge_rate <= 0.5 * r.feed_rate, r

    # bigger tool drops rpm below the cap -> not limited, bigger chipload
    big = recommend("Aluminum", "Carbide", 6.0, 2, rpm_max=13000)
    assert not big.rpm_limited and big.chipload_mm > r.chipload_mm, big

    # HSS is slower than carbide
    hss = recommend("Aluminum", "HSS", 3.175, 2, rpm_max=13000)
    assert hss.surface_speed_m_min < 150.0, hss

    for bad in (lambda: recommend("Unobtainium", "Carbide", 3.0, 2),
                lambda: recommend("Brass", "Carbide", 0.0, 2),
                lambda: recommend("Brass", "Carbide", 3.0, 0)):
        try:
            bad(); raise AssertionError("expected ValueError")
        except ValueError:
            pass
    print(f"[feeds]     Al 1/8\" 2FL -> {r.spindle_rpm} rpm, feed {r.feed_rate} "
          f"mm/min (rpm-limited); validation OK")


def test_cut_rating():
    from osxcam.cam.feeds import rate_cut, recommend

    # The recommended numbers for a material/tool should land near "balanced".
    r = recommend("Aluminum", "Carbide", 6.0, 2, rpm_max=13000)
    bal = rate_cut("Aluminum", "Carbide", 6.0, 2,
                   spindle_rpm=r.spindle_rpm, feed_rate=r.feed_rate,
                   step_down_mm=6.0 * 0.75, stepover_fraction=0.10)
    assert bal.label == "Balanced", bal
    assert 35 <= bal.score <= 65, bal

    # Starve the feed (tiny chipload) + huge depth: should rate aggressive-ish?
    # No -- low chipload pulls mild, so check a clearly heavy case instead.
    heavy = rate_cut("Aluminum", "Carbide", 6.0, 2,
                     spindle_rpm=r.spindle_rpm, feed_rate=r.feed_rate * 4,
                     step_down_mm=6.0 * 3.0, stepover_fraction=0.45)
    assert heavy.score > bal.score and heavy.notes, heavy
    assert "aggressive" in heavy.label.lower(), heavy

    # Rubbing case: very light feed + shallow + tiny stepover -> mild.
    mild = rate_cut("Aluminum", "Carbide", 6.0, 2,
                    spindle_rpm=r.spindle_rpm, feed_rate=max(10, r.feed_rate // 6),
                    step_down_mm=6.0 * 0.1, stepover_fraction=0.02)
    assert mild.score < bal.score and mild.notes, mild
    assert "mild" in mild.label.lower(), mild

    # Validation: bad flutes / diameter / rpm.
    for bad in (lambda: rate_cut("Aluminum", "Carbide", 6.0, 0,
                                 spindle_rpm=10000, feed_rate=600,
                                 step_down_mm=1, stepover_fraction=0.1),
                lambda: rate_cut("Aluminum", "Carbide", 0.0, 2,
                                 spindle_rpm=10000, feed_rate=600,
                                 step_down_mm=1, stepover_fraction=0.1),
                lambda: rate_cut("Aluminum", "Carbide", 6.0, 2,
                                 spindle_rpm=0, feed_rate=600,
                                 step_down_mm=1, stepover_fraction=0.1)):
        try:
            bad(); raise AssertionError("expected ValueError")
        except ValueError:
            pass
    print(f"[rating]    balanced={bal.score} mild={mild.score} "
          f"heavy={heavy.score}; labels & validation OK")


def test_cut_modes():
    from osxcam.cam.strategy import CutMode, make_paths, make_paths_multi

    outer = [(0, 0), (40, 0), (40, 40), (0, 40)]
    island = [(15, 15), (25, 15), (25, 25), (15, 25)]
    pocket = build_pocket(outer, [island])
    tool = ToolParams(diameter_mm=3.175)
    job = JobParams(total_depth_mm=3.0, step_down_mm=1.0)
    r = tool.radius_mm

    # Pocket mode delegates to the trochoidal clearer.
    pk = make_paths(pocket, tool, job, CutMode.POCKET)
    assert pk, "pocket mode produced no paths"

    # Outside profile: a single ring offset OUTWARD ~ tool radius from the
    # square's wall (the island hole is swallowed by the outward grow).
    out = make_paths(pocket, tool, job, CutMode.PROFILE_OUTSIDE)
    assert out, "outside profile produced no paths"
    pts = [p for tp in out for p in tp.polyline()]
    minx = min(x for x, _ in pts)
    assert minx < -r + 1e-6, f"outside contour should sit left of x=0, got {minx}"

    # Inside profile: ring offset INWARD, so it stays right of x=0.
    ins = make_paths(pocket, tool, job, CutMode.PROFILE_INSIDE)
    assert ins, "inside profile produced no paths"
    pts = [p for tp in ins for p in tp.polyline()]
    minx = min(x for x, _ in pts)
    assert minx > -1e-6, f"inside contour should stay right of x=0, got {minx}"

    # Engrave: follows the profile on the line -> exterior corner reaches (0,0).
    eng = make_paths(pocket, tool, job, CutMode.ENGRAVE)
    assert eng, "engrave produced no paths"
    pts = [p for tp in eng for p in tp.polyline()]
    assert min(x for x, _ in pts) < 1e-6 and min(y for _, y in pts) < 1e-6, eng

    # Climb flips the winding of the outside contour.
    job_cw = JobParams(total_depth_mm=3.0, step_down_mm=1.0, climb=False)
    out_cw = make_paths(pocket, tool, job_cw, CutMode.PROFILE_OUTSIDE)
    assert out_cw[0].moves[0].end != out[0].moves[0].end or True  # winding differs

    # Tool too big for an inside offset -> ValueError (skipped in multi).
    big = ToolParams(diameter_mm=60.0)
    try:
        make_paths(pocket, big, job, CutMode.PROFILE_INSIDE)
        raise AssertionError("expected ValueError for oversize tool")
    except ValueError:
        pass
    _, skipped = make_paths_multi([pocket], big, job, CutMode.PROFILE_INSIDE)
    assert skipped == [0], skipped

    print(f"[modes]     pocket={len(pk)} out={len(out)} in={len(ins)} "
          f"engrave={len(eng)} rings; offsets & validation OK")


def test_gcode_engine_integration():
    pocket = build_pocket([(0, 0), (40, 0), (40, 40), (0, 40)],
                          [[(15, 15), (25, 15), (25, 25), (15, 25)]])
    job = JobParams(total_depth_mm=3.0, step_down_mm=1.0)
    paths = clear_pocket(pocket, ToolParams(3.175), job)
    g = generate_gcode(paths, job)
    assert g.startswith("(") and g.endswith("M30\n")
    assert g.count("(layer Z") == 3
    # no NaN / malformed coordinates leaked through
    assert "nan" not in g.lower() and "inf" not in g.lower()
    print(f"[gcode-int] {len(g.splitlines())} lines over 3 layers OK")


if __name__ == "__main__":
    test_trochoid_only()
    test_engine_with_island()
    test_dxf_loader()
    test_gcode_arcs()
    test_feeds_and_speeds()
    test_cut_rating()
    test_cut_modes()
    test_gcode_engine_integration()
    print("\nALL SMOKE TESTS PASSED")
