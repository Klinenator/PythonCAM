"""Tool and job parameters for a trochoidal clearing operation.

All lengths are millimetres, feeds are mm/min. The derived properties below
encode the trochoidal model used by the engine:

  * ``stepover_mm``  - target radial engagement into uncut material. The loop
                       advance (``pitch_mm``) is set equal to this so the fresh
                       sliver cut on the leading edge of each loop never exceeds
                       it, which is what bounds the engagement angle.
  * ``loop_radius_mm`` - radius of the circular tool-CENTRE motion. The slot a
                       single guide ring clears is ``2*loop_radius + diameter``
                       wide. Defaults to one tool diameter.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ToolParams:
    diameter_mm: float

    def __post_init__(self) -> None:
        if self.diameter_mm <= 0:
            raise ValueError("tool diameter must be positive")

    @property
    def radius_mm(self) -> float:
        return self.diameter_mm / 2.0


@dataclass(frozen=True)
class StockParams:
    """Raw material blank. Origin is the lower-left corner at (0, 0); the top
    surface is the Z reference (``JobParams.top_z_mm``)."""

    width_mm: float
    height_mm: float
    thickness_mm: float

    def __post_init__(self) -> None:
        if min(self.width_mm, self.height_mm, self.thickness_mm) <= 0:
            raise ValueError("stock dimensions must be positive")

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (0.0, 0.0, self.width_mm, self.height_mm)

    def contains_bounds(self, minx: float, miny: float,
                        maxx: float, maxy: float, tol: float = 1e-6) -> bool:
        return (minx >= -tol and miny >= -tol
                and maxx <= self.width_mm + tol
                and maxy <= self.height_mm + tol)


@dataclass(frozen=True)
class JobParams:
    total_depth_mm: float          # final depth below the top of stock (positive)
    step_down_mm: float            # max material removed per Z layer
    stepover_fraction: float = 0.12  # radial engagement as a fraction of diameter
    feed_rate: float = 600.0       # cutting feed, mm/min (G1/G2/G3)
    plunge_rate: float = 150.0     # Z plunge feed, mm/min
    spindle_rpm: float = 10000.0   # spindle speed for M3 S...
    safe_z_mm: float = 5.0         # rapid/retract height above stock top
    top_z_mm: float = 0.0          # Z of the stock top surface
    loop_radius_mm: float | None = None  # None -> default to tool diameter
    climb: bool = True             # climb milling -> CCW loops for an inner pocket
    helix_ramp_angle_deg: float = 3.0  # plunge ramp angle; 0 -> straight plunge

    def __post_init__(self) -> None:
        if self.total_depth_mm <= 0:
            raise ValueError("total depth must be positive")
        if self.step_down_mm <= 0:
            raise ValueError("step down must be positive")
        if not 0.0 < self.stepover_fraction <= 0.5:
            raise ValueError("stepover_fraction must be in (0, 0.5]")
        if self.safe_z_mm <= self.top_z_mm:
            raise ValueError("safe_z must be above top_z")
        if not 0.0 <= self.helix_ramp_angle_deg < 90.0:
            raise ValueError("helix_ramp_angle_deg must be in [0, 90)")

    def stepover_mm(self, tool: ToolParams) -> float:
        return self.stepover_fraction * tool.diameter_mm

    def pitch_mm(self, tool: ToolParams) -> float:
        """Advance of the loop centre per revolution (== target engagement)."""
        return self.stepover_mm(tool)

    def loop_radius(self, tool: ToolParams) -> float:
        return self.loop_radius_mm if self.loop_radius_mm else tool.diameter_mm

    def z_layers(self) -> list[float]:
        """Absolute Z for the bottom of each pass, top -> final depth."""
        layers: list[float] = []
        z = self.top_z_mm
        bottom = self.top_z_mm - self.total_depth_mm
        while z - self.step_down_mm > bottom + 1e-9:
            z -= self.step_down_mm
            layers.append(z)
        layers.append(bottom)
        return layers
