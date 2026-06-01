"""Entry point: launch the osxCAM desktop UI.

    ./.venv/bin/python -m osxcam.main      (from the src/ dir, or with src on PATH)
    ./.venv/bin/python src/osxcam/main.py
"""

from __future__ import annotations

import os
import sys

# allow running the file directly without installing the package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from osxcam.ui.app import main  # noqa: E402

if __name__ == "__main__":
    main()
