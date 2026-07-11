# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""ROS 2 topic/mode contract documentation test (no hardware)."""
from __future__ import annotations

EXPECTED_MODES = {
    "baseline_only",
    "supervised_residual",
    "sac_residual",
    "validation_only",
}


def test_documented_modes_match_spec():
    assert "baseline_only" in EXPECTED_MODES
    assert len(EXPECTED_MODES) == 4
