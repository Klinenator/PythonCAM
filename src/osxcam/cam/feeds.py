"""Feeds & speeds estimator: material + tool -> spindle RPM and feed rates.

These are deliberately *conservative* starting points tuned for a hobby GRBL
router (far less rigid than a production mill), not aggressive shop numbers. The
UI fills them as editable suggestions -- always verify by chip/sound and adjust.

Model (metric)
--------------
* Spindle RPM from surface speed Vc (material x tool material):
      rpm = Vc * 1000 / (pi * diameter)        clamped to the spindle's range
* Feed from chipload fz (material, scaled by diameter):
      feed = rpm * flutes * fz
* Plunge ~= 0.4 * feed.

When the ideal RPM exceeds the spindle's max (common for small tools in metal),
the cut is *RPM-limited*: we clamp RPM down, which lowers the achievable surface
speed and feed. The result flags this so the UI can say so.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

D_REF = 3.175  # reference tool diameter (1/8") the chiploads are quoted at

# material -> (Vc m/min for carbide, chipload mm/tooth at D_REF for carbide)
MATERIALS: dict[str, tuple[float, float]] = {
    "Aluminum": (150.0, 0.025),
    "Brass": (120.0, 0.040),
    "Wood / MDF": (350.0, 0.100),
    "Acrylic / plastic": (250.0, 0.050),
}

TOOL_MATERIALS = ("Carbide", "HSS")
_HSS_VC_FACTOR = 0.5        # HSS runs slower than carbide
_HSS_CHIPLOAD_FACTOR = 0.8

# Default Maxmake spindle range (max confirmed by the user = 13000 rpm).
DEFAULT_RPM_MIN = 8000.0
DEFAULT_RPM_MAX = 13000.0


@dataclass(frozen=True)
class FeedsResult:
    spindle_rpm: int        # rpm, rounded to 100
    feed_rate: int          # mm/min, rounded to 10
    plunge_rate: int        # mm/min, rounded to 10
    chipload_mm: float      # feed per tooth used
    surface_speed_m_min: float  # Vc actually achieved at the (clamped) rpm
    rpm_limited: bool       # True if ideal rpm exceeded the spindle max


# Balanced references for the *cut geometry* (independent of the feeds model).
# Trochoidal adaptive clearing wants a light radial bite but is happy with a
# deep axial slice -- that's the whole point of the strategy.
_RADIAL_BALANCED = 0.10     # stepover as a fraction of diameter
_AXIAL_BALANCED = 0.75      # depth of cut as a multiple of diameter


@dataclass(frozen=True)
class CutRating:
    label: str              # "Very mild" .. "Balanced" .. "Very aggressive"
    score: int              # 0 = mildest, 50 ~= balanced, 100 = most aggressive
    notes: list[str]        # human-readable cautions (both extremes are bad)


def _reference(material: str, tool_material: str,
               diameter_mm: float) -> tuple[float, float]:
    """Recommended (surface speed Vc m/min, chipload mm/tooth) for this combo."""
    if material not in MATERIALS:
        raise ValueError(f"unknown material: {material!r}")
    if diameter_mm <= 0:
        raise ValueError("tool diameter must be positive")
    vc, chip_ref = MATERIALS[material]
    if tool_material == "HSS":
        vc *= _HSS_VC_FACTOR
        chip_ref *= _HSS_CHIPLOAD_FACTOR
    # bigger tools take bigger chips; scale chipload with diameter (bounded)
    scale = min(2.5, max(0.4, diameter_mm / D_REF))
    return vc, chip_ref * scale


def recommend(material: str, tool_material: str, diameter_mm: float,
              flutes: int, rpm_min: float = DEFAULT_RPM_MIN,
              rpm_max: float = DEFAULT_RPM_MAX) -> FeedsResult:
    if flutes < 1:
        raise ValueError("flutes must be >= 1")
    if rpm_max <= 0 or rpm_min <= 0 or rpm_min > rpm_max:
        raise ValueError("invalid spindle rpm range")

    vc, chipload = _reference(material, tool_material, diameter_mm)

    rpm_ideal = vc * 1000.0 / (math.pi * diameter_mm)
    rpm = min(rpm_max, max(rpm_min, rpm_ideal))
    rpm_limited = rpm_ideal > rpm_max

    feed = rpm * flutes * chipload
    plunge = 0.4 * feed
    achieved_vc = math.pi * diameter_mm * rpm / 1000.0

    return FeedsResult(
        spindle_rpm=int(round(rpm / 100.0) * 100),
        feed_rate=int(round(feed / 10.0) * 10),
        plunge_rate=int(round(plunge / 10.0) * 10),
        chipload_mm=round(chipload, 4),
        surface_speed_m_min=round(achieved_vc, 1),
        rpm_limited=rpm_limited,
    )


def _stops(actual: float, reference: float) -> float:
    """Signed log2 deviation ("stops") of actual from reference.

    +1 means double the reference, -1 means half. Symmetric in a way a plain
    ratio is not, which is what we want: too-light and too-heavy should be
    treated as mirror images, both walking away from the balanced middle.
    """
    if actual <= 0.0 or reference <= 0.0:
        return 0.0
    return math.log2(actual / reference)


def rate_cut(material: str, tool_material: str, diameter_mm: float,
             flutes: int, spindle_rpm: float, feed_rate: float,
             step_down_mm: float, stepover_fraction: float,
             rpm_max: float = DEFAULT_RPM_MAX) -> CutRating:
    """Rate how aggressive the *entered* parameters are, mild..aggressive.

    Looks at everything the user has dialled in -- chipload (from feed, rpm and
    flutes), surface speed (from rpm and diameter), radial engagement (stepover)
    and axial depth -- and compares each against a balanced reference for the
    material/tool. Both extremes are called out: too mild rubs and overheats the
    edge (poor tool life, work-hardening in alloys); too aggressive overloads
    the flute and risks chipping/snapping. ~50 is the sweet spot.
    """
    if flutes < 1:
        raise ValueError("flutes must be >= 1")
    if diameter_mm <= 0:
        raise ValueError("tool diameter must be positive")
    if spindle_rpm <= 0:
        raise ValueError("spindle rpm must be positive")

    vc_ideal, fz_ideal = _reference(material, tool_material, diameter_mm)

    # What the entered numbers actually imply at the spindle.
    fz = feed_rate / (spindle_rpm * flutes)         # chipload, mm/tooth
    vc = math.pi * diameter_mm * spindle_rpm / 1000.0   # surface speed, m/min
    radial_frac = stepover_fraction
    axial_ratio = step_down_mm / diameter_mm

    chip_dev = _stops(fz, fz_ideal)
    speed_dev = _stops(vc, vc_ideal)
    radial_dev = _stops(radial_frac, _RADIAL_BALANCED)
    axial_dev = _stops(axial_ratio, _AXIAL_BALANCED)

    # Chipload dominates edge load; radial bite next; axial less so for a
    # trochoidal strategy; surface speed is mostly a heat/finish lever.
    overall = (0.35 * chip_dev + 0.30 * radial_dev
               + 0.25 * axial_dev + 0.10 * speed_dev)

    # +/-2 stops (4x off balanced) saturates the 0..100 dial.
    score = int(round(max(0.0, min(100.0, 50.0 + overall * 25.0))))

    if overall <= -1.0:
        label = "Very mild"
    elif overall <= -0.35:
        label = "Mild"
    elif overall < 0.35:
        label = "Balanced"
    elif overall < 1.0:
        label = "Aggressive"
    else:
        label = "Very aggressive"

    notes: list[str] = []
    if chip_dev <= -1.0:
        notes.append(
            f"Chipload {fz:.3f} mm/tooth is well below the ~{fz_ideal:.3f} "
            "target -- thin chips rub and overheat the edge (and work-harden "
            "aluminium). Raise feed or drop RPM.")
    elif chip_dev >= 1.0:
        notes.append(
            f"Chipload {fz:.3f} mm/tooth is well above the ~{fz_ideal:.3f} "
            "target -- heavy chips risk chipping or snapping the cutter. "
            "Lower feed or raise RPM.")
    if radial_frac > 0.20:
        notes.append(
            f"Radial engagement {radial_frac*100:.0f}% of diameter is heavy "
            "for a trochoidal clear; keep it light (~10%) to stay cool.")
    elif radial_frac < 0.04:
        notes.append(
            f"Radial engagement {radial_frac*100:.0f}% is very light -- safe "
            "but slow; you can afford more bite.")
    if axial_ratio > 2.0:
        notes.append(
            f"Depth {axial_ratio:.1f}x diameter per pass is deep -- fine for "
            "trochoidal slots if the tool is long enough, but watch deflection.")
    elif axial_ratio < 0.25:
        notes.append(
            f"Depth {axial_ratio:.2f}x diameter is shallow -- lots of passes; "
            "you can go deeper with this strategy.")
    if speed_dev <= -0.7:
        notes.append(
            f"Surface speed {vc:.0f} m/min is well under the ~{vc_ideal:.0f} "
            "target (RPM-limited?) -- expect a rougher finish.")
    elif speed_dev >= 0.6:
        notes.append(
            f"Surface speed {vc:.0f} m/min is high vs ~{vc_ideal:.0f} -- watch "
            "for heat; keep chips evacuating.")

    return CutRating(label=label, score=score, notes=notes)
