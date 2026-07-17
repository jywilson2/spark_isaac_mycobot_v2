# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Gates for SKIPPED_UNREACHABLE fraction and countable-episode semantics."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from isaac_sim.viz_plan_policy import (
    meets_max_skip_unreachable_frac,
    skip_unreachable_frac,
)
from residual_adaptive_ik.geometry.collision import proximal_arm_contacts_target
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad

REPO = Path(__file__).resolve().parents[1]


def test_skip_unreachable_frac_and_gate():
    assert skip_unreachable_frac(0, 0) == 0.0
    assert abs(skip_unreachable_frac(1, 4) - 0.25) < 1e-12
    assert meets_max_skip_unreachable_frac(1, 4, max_frac=0.25) is True
    assert meets_max_skip_unreachable_frac(2, 4, max_frac=0.25) is False
    # Disabled when max_frac >= 1 or < 0.
    assert meets_max_skip_unreachable_frac(99, 100, max_frac=1.0) is True
    assert meets_max_skip_unreachable_frac(99, 100, max_frac=-1.0) is True


def test_collision_yaml_declares_skip_unreachable_gate():
    text = (REPO / "configs" / "planning" / "collision.yaml").read_text(
        encoding="utf-8"
    )
    assert "max_skip_unreachable_frac" in text
    assert "prefer_dexterous_region_candidates" in text


def test_run_ik_viz_excludes_skips_from_episode_count():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "n_countable" in src
    assert "Dexterous Region only" in src
    assert "dropped_out_of_region" in src
    assert "not counted as an episode" in src or "excluded from episode count" in src
    assert "meets_max_skip_unreachable_frac" in src
    assert "proximal_arm_contacts_target" in src
    assert "MARKER_ARM_BODY" in src
    assert "prefer Dexterous Region" in src


def test_proximal_arm_contacts_target_home_clear_of_far_marker():
    """At home, a far-away marker must not collide with proximal capsules."""
    q = load_home_joint_positions_rad()
    tip = np.asarray(forward_kinematics(q).position_m, dtype=float).reshape(3)
    far = tip + np.array([0.5, 0.0, 0.0], dtype=float)
    report = proximal_arm_contacts_target(q, far, target_radius_m=0.012)
    assert report.collides is False


def test_proximal_arm_contacts_target_detects_center_hit():
    """A marker centered on a proximal chain point must be reported."""
    q = load_home_joint_positions_rad()
    # Place the marker at the origin-ish base/proximal region so coarse
    # capsules from home almost always intersect (base column).
    report = proximal_arm_contacts_target(
        q,
        np.array([0.0, 0.0, 0.10], dtype=float),
        target_radius_m=0.05,
    )
    assert report.collides is True
    assert report.reasons


def test_proximal_arm_contacts_ignores_distal_ee_chain_at_tip():
    """Valid tip-face proximity must not false-positive on wrist capsules."""
    q = load_home_joint_positions_rad()
    tip = np.asarray(forward_kinematics(q).position_m, dtype=float).reshape(3)
    # Marker centered just beyond the tip (surface contact geometry) — distal
    # EE capsules would hit with fat radii, but proximal monitor must stay clear.
    near_tip = tip + np.array([0.0, 0.0, 0.012], dtype=float)
    report = proximal_arm_contacts_target(
        q, near_tip, target_radius_m=0.012, n_ee_segments_ignored=3
    )
    assert report.collides is False
