# osxCAM

A lightweight macOS desktop **2.5D CAM** application. Import a 2D vector/CAD
file, pick an enclosed profile, generate a metal-ready **trochoidal (adaptive
clearing)** pocketing toolpath, and export **GRBL G-code** for a hobby CNC
router.

> Conservative, hobby-router-tuned defaults. Every suggested number is an
> editable starting point — always verify by chip/sound and adjust.

## Features

- **Import** SVG, DXF, and STEP profiles (STEP requires the conda runtime — see
  below).
- **Selectable cut modes** — pick a machining strategy per job:
  - *Pocket (adaptive)* — trochoidal area clearing (below).
  - *Profile — outside* — contour offset outward by the tool radius (cut a part
    free from stock).
  - *Profile — inside* — contour offset inward (cut a hole/window to size).
  - *Engrave (on line)* — follow the profile centerline with no offset.
- **Trochoidal adaptive clearing** with nested guide rings, climb/conventional
  selection, and auto-fit loop radius for small pockets.
- **Helical plunge entry** (configurable ramp angle) instead of straight
  plunges.
- **Feeds & speeds estimator** — material + tool material + flute count suggest
  spindle RPM, feed, and plunge (RPM clamped to the spindle's range).
- **Cut aggressiveness rating** — scores the entered parameters from mild to
  aggressive, flagging both extremes for tool longevity.
- **2D toolpath preview** with a **cut simulation** that plays in real machining
  time (1×) with speed multipliers.
- **3D part/stock view** — rotatable preview of the stock block with the
  pocket(s) cut to depth, including a 3D simulation that descends through the Z
  layers.
- **GRBL G-code export** (G2/G3 arcs, per-layer depth passes).

## Target machine

Output targets a **GRBL** controller (developed against a Maxmake router).
Z convention: **Z0 = top of stock**, cutting downward to negative Z. Tool
touch-off / probing is left to the sender, so the exported file stays a pure
toolpath.

## Requirements

Python 3.11 or 3.12 recommended (Shapely 2.x ships wheels for CPython 3.9–3.13).

Core dependencies (`requirements.txt`):

- shapely ≥ 2.0 — geometry engine (offsetting, booleans, islands)
- ezdxf ≥ 1.1 — DXF parsing
- svgelements ≥ 1.9 — SVG parsing
- customtkinter ≥ 5.2, matplotlib ≥ 3.8 — UI and preview

## Setup & running

Works on **macOS, Linux, and Windows** — all dependencies are cross-platform.
The only difference is the virtualenv layout and how environment variables are
set on the command line.

### SVG / DXF (pip virtualenv)

**macOS / Linux**

```bash
python3.12 -m venv .venv
./.venv/bin/pip install -r requirements.txt
PYTHONPATH=src ./.venv/bin/python -m osxcam.main
```

**Windows (PowerShell)**

```powershell
py -3.12 -m venv .venv
.venv\Scripts\pip install -r requirements.txt
$env:PYTHONPATH = "src"; .venv\Scripts\python -m osxcam.main
```

**Windows (cmd.exe)**

```cmd
py -3.12 -m venv .venv
.venv\Scripts\pip install -r requirements.txt
set PYTHONPATH=src && .venv\Scripts\python -m osxcam.main
```

### STEP import (conda)

STEP import needs `pythonocc-core`, which is only available via conda (not pip
or Homebrew). It's available for macOS, Linux, and Windows on conda-forge.
Create a conda env and run from it:

**macOS / Linux**

```bash
conda create -n osxcam python=3.11
conda activate osxcam
conda install -c conda-forge pythonocc-core=7.9.0
pip install -r requirements.txt
PYTHONPATH=src python -m osxcam.main
```

**Windows (PowerShell / Anaconda Prompt)**

```powershell
conda create -n osxcam python=3.11
conda activate osxcam
conda install -c conda-forge pythonocc-core=7.9.0
pip install -r requirements.txt
$env:PYTHONPATH = "src"; python -m osxcam.main
```

## Tests

A smoke suite exercises the geometry engine, loaders, G-code, feeds/speeds, and
cut rating:

```bash
# macOS / Linux
PYTHONPATH=src python tests/smoke_engine.py
```

```powershell
# Windows (PowerShell)
$env:PYTHONPATH = "src"; python tests/smoke_engine.py
```

## Project layout

```
src/osxcam/
  cam/        feeds & speeds, params, cut-mode strategies, toolpaths
  geometry/   region offsetting, guide rings, trochoidal path generation
  gcode/      GRBL post-processor
  io/         file loaders (SVG/DXF/STEP)
  ui/         CustomTkinter app, 2D canvas + simulation, 3D view
  main.py     entry point
tests/        smoke tests
```

## License

[MIT](LICENSE)
