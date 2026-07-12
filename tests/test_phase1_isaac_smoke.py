# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Deprecated alias — prefer ``tests/test_isaac_viz_smoke.py``."""
from __future__ import annotations

from pathlib import Path


def test_legacy_phase1_smoke_module_points_at_isaac_viz():
    sibling = Path(__file__).resolve().parent / "test_isaac_viz_smoke.py"
    assert sibling.is_file()
    text = sibling.read_text(encoding="utf-8")
    assert "smoke_isaac_viz.sh" in text
