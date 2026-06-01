"""GRBL G-code generation from a list of XY :class:`Toolpath` rings.

Each ring is replayed at every Z layer: rapid to safe Z, rapid over the start,
plunge at the plunge feed, cut the loops (G1/G2/G3, Z held constant), retract.
Arc IJK are incremental from the arc start to the centre stored on the move, so
position is tracked as lines are emitted.

Targets GRBL v1.1: G21 metric, G90 absolute, G17 XY arc plane, G94 feed/min,
incremental IJK arcs (GRBL's only mode), a spindle spin-up dwell after M3, and
M30 to end. Comments use parentheses.
"""

from __future__ import annotations

import math

from ..cam.params import JobParams, ToolParams
from ..cam.toolpath import MoveType, Toolpath

FILE_EXT = ".nc"
SPINDLE_DWELL_S = 1.0  # G4 dwell after M3 to let the spindle reach speed
_ARC = (MoveType.ARC_CW, MoveType.ARC_CCW)


def _fmt(v: float, precision: int) -> str:
    s = f"{v:.{precision}f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


class _Writer:
    def __init__(self, precision: int) -> None:
        self.precision = precision
        self.lines: list[str] = []
        self.x: float | None = None
        self.y: float | None = None
        self.z: float | None = None
        self.f: float | None = None

    def raw(self, line: str) -> None:
        self.lines.append(line)

    def comment(self, text: str) -> None:
        self.lines.append(f"({text})")

    def _n(self, v: float) -> str:
        return _fmt(v, self.precision)

    def _changed(self, cur: float | None, new: float | None) -> bool:
        return new is not None and (cur is None or abs(cur - new) > 1e-9)

    def move(self, code: str, x=None, y=None, z=None, f=None,
             i=None, j=None) -> None:
        words = [code]
        if self._changed(self.x, x):
            words.append(f"X{self._n(x)}")
            self.x = x
        if self._changed(self.y, y):
            words.append(f"Y{self._n(y)}")
            self.y = y
        if self._changed(self.z, z):
            words.append(f"Z{self._n(z)}")
            self.z = z
        if i is not None:
            words.append(f"I{self._n(i)}")
        if j is not None:
            words.append(f"J{self._n(j)}")
        if f is not None and self._changed(self.f, f):
            words.append(f"F{self._n(f)}")
            self.f = f
        if len(words) > 1:  # skip no-op moves
            self.lines.append(" ".join(words))


def _first_loop(tp: Toolpath):
    """Return ``(m0, m1, radius)`` if the path opens with a full circular loop
    (the two semicircle arcs the trochoid emits), else ``None``.

    The helix descends by re-tracing this loop, so it is guaranteed to lie on a
    circle the engine already proved fits inside the machinable region.
    """
    moves = tp.moves
    if len(moves) < 2:
        return None
    m0, m1 = moves[0], moves[1]
    if m0.kind not in _ARC or m1.kind not in _ARC:
        return None
    if m0.center is None or m1.center is None:
        return None
    sx, sy = tp.start
    if abs(m1.end[0] - sx) > 1e-6 or abs(m1.end[1] - sy) > 1e-6:
        return None  # second arc must close back on the start
    r = math.hypot(m0.center[0] - sx, m0.center[1] - sy)
    if r <= 1e-6:
        return None
    return m0, m1, r


def _emit_helix(w: "_Writer", tp: Toolpath, loop: tuple,
                top_z: float, target_z: float, job: JobParams) -> bool:
    """Helix down from ``top_z`` to ``target_z`` along the first loop.

    Returns ``False`` (caller falls back to a straight plunge) if the ramp angle
    is zero or no descent is needed.
    """
    drop = top_z - target_z
    if drop <= 1e-9 or job.helix_ramp_angle_deg <= 0.0:
        return False
    m0, m1, radius = loop
    z_per_rev = 2.0 * math.pi * radius * math.tan(
        math.radians(job.helix_ramp_angle_deg))
    if z_per_rev <= 1e-9:
        return False

    revs = max(1, math.ceil(drop / z_per_rev))
    half_steps = revs * 2
    cur = tp.start
    step = 0
    for _ in range(revs):
        for m in (m0, m1):
            step += 1
            z = top_z - drop * (step / half_steps)
            code = "G3" if m.kind is MoveType.ARC_CCW else "G2"
            w.move(code, x=m.end[0], y=m.end[1], z=z,
                   i=m.center[0] - cur[0], j=m.center[1] - cur[1],
                   f=job.plunge_rate)
            cur = m.end
    return True


def generate_gcode(paths: list[Toolpath], job: JobParams,
                   tool: ToolParams | None = None,
                   program_name: str = "osxCAM trochoidal pocket",
                   precision: int = 4) -> str:
    w = _Writer(precision)
    safe = job.safe_z_mm

    w.comment(program_name)
    if tool is not None:
        w.comment(f"tool dia {tool.diameter_mm} mm")
    w.comment(f"GRBL | depth {job.total_depth_mm} mm "
              f"in {len(job.z_layers())} pass(es)")

    w.raw("G21")   # millimetres
    w.raw("G90")   # absolute
    w.raw("G17")   # XY arc plane
    w.raw("G94")   # units/min feed
    w.raw(f"M3 S{int(job.spindle_rpm)}")
    w.raw(f"G4 P{_fmt(SPINDLE_DWELL_S, precision)}")
    w.move("G0", z=safe)

    prev_z = job.top_z_mm
    for z in job.z_layers():
        w.comment(f"layer Z{_fmt(z, precision)}")
        for tp in paths:
            sx, sy = tp.start
            w.move("G0", z=safe)
            w.move("G0", x=sx, y=sy)

            loop = _first_loop(tp)
            if loop is not None:
                w.move("G0", z=prev_z)  # rapid to top of this layer's material
                if not _emit_helix(w, tp, loop, prev_z, z, job):
                    w.move("G1", z=z, f=job.plunge_rate)
            else:
                w.move("G1", z=z, f=job.plunge_rate)

            cur = tp.start
            for m in tp.moves:
                ex, ey = m.end
                if m.kind in (MoveType.LINEAR, MoveType.RAPID):
                    code = "G0" if m.kind is MoveType.RAPID else "G1"
                    w.move(code, x=ex, y=ey, f=job.feed_rate)
                else:
                    assert m.center is not None
                    i = m.center[0] - cur[0]
                    j = m.center[1] - cur[1]
                    code = "G3" if m.kind is MoveType.ARC_CCW else "G2"
                    w.move(code, x=ex, y=ey, i=i, j=j, f=job.feed_rate)
                cur = m.end

            w.move("G0", z=safe)
        prev_z = z

    w.raw("M5")
    w.move("G0", z=safe)
    w.raw("M30")
    return "\n".join(w.lines) + "\n"


def write_gcode_file(filepath: str, paths: list[Toolpath], job: JobParams,
                     tool: ToolParams | None = None) -> None:
    with open(filepath, "w") as fh:
        fh.write(generate_gcode(paths, job, tool=tool))
