"""Machine-independent 2D toolpath model.

A :class:`Toolpath` is an ordered list of :class:`Move` objects in the XY plane.
Z handling (layers, plunges, retracts) is layered on top by the G-code stage,
so the same XY toolpath can be replayed at every depth.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..geometry.primitives import Point, sample_arc


class MoveType(Enum):
    RAPID = "G0"
    LINEAR = "G1"
    ARC_CW = "G2"
    ARC_CCW = "G3"


@dataclass(frozen=True)
class Move:
    kind: MoveType
    end: Point
    center: Point | None = None  # absolute arc centre, required for G2/G3

    @property
    def is_arc(self) -> bool:
        return self.kind in (MoveType.ARC_CW, MoveType.ARC_CCW)


class Toolpath:
    def __init__(self, start: Point) -> None:
        self.start: Point = start
        self.moves: list[Move] = []

    @property
    def end(self) -> Point:
        return self.moves[-1].end if self.moves else self.start

    def linear(self, end: Point) -> None:
        self.moves.append(Move(MoveType.LINEAR, end))

    def rapid(self, end: Point) -> None:
        self.moves.append(Move(MoveType.RAPID, end))

    def arc(self, end: Point, center: Point, ccw: bool) -> None:
        self.moves.append(
            Move(MoveType.ARC_CCW if ccw else MoveType.ARC_CW, end, center)
        )

    def extend(self, other: "Toolpath") -> None:
        self.moves.extend(other.moves)

    def polyline(self, max_step_deg: float = 6.0) -> list[Point]:
        """Flatten to points (arcs sampled) for the 2D preview canvas.

        ``max_step_deg`` controls arc resolution; the preview passes a coarse
        value to keep huge jobs responsive (the G-code emits true arcs and never
        uses this).
        """
        pts: list[Point] = [self.start]
        cur = self.start
        for m in self.moves:
            if m.is_arc:
                assert m.center is not None
                pts.extend(sample_arc(cur, m.end, m.center,
                                      ccw=m.kind is MoveType.ARC_CCW,
                                      max_step_deg=max_step_deg))
            else:
                pts.append(m.end)
            cur = m.end
        return pts
