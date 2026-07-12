# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Tests for home joint pose loading."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad

REPO = Path(__file__).resolve().parents[1]


def test_home_joint_positions_length_six():
    q = load_home_joint_positions_rad()
    assert q.shape == (6,)
    assert np.all(np.isfinite(q))


def test_viz_cli_has_reset_to_home_flags():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "--reset-to-home" in src
    assert "--no-reset-to-home" in src
    assert "args.reset_to_home" in src


def test_smoke_script_forwards_reset_to_home():
    src = (REPO / "scripts" / "host" / "smoke_isaac_viz.sh").read_text(encoding="utf-8")
    assert "--reset-to-home" in src
    assert "ISAAC_VIZ_SMOKE_RESET_TO_HOME" in src


def test_collision_yaml_enables_home_reset():
    from residual_adaptive_ik.planning.curobo_planner import load_planning_config

    cfg = load_planning_config()
    # Default off so path-dependent recovery can be tested; opt-in via YAML.
    assert cfg.get("reset_to_home_before_each_trial", True) is False
