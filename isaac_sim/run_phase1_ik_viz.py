#!/usr/bin/env python3
# Deprecated alias — use isaac_sim/run_ik_viz.py
"""Deprecated wrapper; prefer ``isaac_sim/run_ik_viz.py``."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

print(
    "NOTE: run_phase1_ik_viz.py is deprecated; forwarding to run_ik_viz.py",
    file=sys.stderr,
)
runpy.run_path(str(Path(__file__).resolve().parent / "run_ik_viz.py"), run_name="__main__")
