# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Contracts for multi-seed IK reseeding (Phase 2 sequential multi-target)."""
from __future__ import annotations

import numpy as np
import pytest

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.ik_seed_bank import (
    build_seed_bank,
    joint_distance_rad,
    order_seeds,
    solve_with_seed_bank,
)
from residual_adaptive_ik.kinematics.numerical_ik import DampedLeastSquaresIK
from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad


def test_build_seed_bank_includes_current_and_home():
    home = load_home_joint_positions_rad()
    q_cur = home + np.array([0.2, -0.1, 0.05, 0.0, 0.1, -0.05])
    bank = build_seed_bank(q_cur, home_q=home, previous_goals=[home + 0.3])
    assert len(bank) >= 3
    assert np.allclose(bank[0], q_cur)
    assert any(np.allclose(s, home) for s in bank)


def test_order_seeds_prefers_near_current_before_failures():
    q_cur = np.zeros(6)
    near = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    far = np.array([1.0, 0.5, 0.0, 0.0, 0.0, 0.0])
    ordered = order_seeds([far, near], q_current=q_cur, failed_seeds=None)
    assert np.allclose(ordered[0], near)


def test_order_seeds_after_failure_prefers_far_from_failed():
    """IKSel-style: escape the failed basin in joint space (not tip distance)."""
    q_cur = np.zeros(6)
    failed = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
    near_fail = np.array([0.08, 0.0, 0.0, 0.0, 0.0, 0.0])
    far_fail = np.array([1.2, 0.4, 0.0, 0.0, 0.0, 0.0])
    ordered = order_seeds(
        [near_fail, far_fail, failed],
        q_current=q_cur,
        failed_seeds=[failed],
    )
    assert len(ordered) == 2
    assert np.allclose(ordered[0], far_fail)
    assert joint_distance_rad(ordered[0], failed) > joint_distance_rad(
        ordered[1], failed
    )


def test_solve_with_seed_bank_reaches_reachable_pose():
    solver = DampedLeastSquaresIK()
    home = load_home_joint_positions_rad()
    q_true = home + np.array([0.3, -0.2, 0.15, 0.1, -0.1, 0.05])
    target = forward_kinematics(q_true)
    # Bad first seed (zeros) plus a good mid-blend / true-ish seed.
    seeds = build_seed_bank(
        home,
        home_q=home,
        extra_seeds=[q_true],
        include_mid_home=True,
    )
    result = solve_with_seed_bank(
        solver, target, seeds, q_current=home, max_seeds=6
    )
    assert result.success, result.reason
    assert result.position_error_m < 2e-3
    assert "seed_bank_ok" in result.reason


def test_solve_with_seed_bank_empty_reports_failure():
    solver = DampedLeastSquaresIK()
    target = Pose(
        position_m=np.array([10.0, 10.0, 10.0]),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    result = solve_with_seed_bank(
        solver, target, [np.zeros(6)], q_current=np.zeros(6), max_seeds=1
    )
    assert not result.success
    assert "seed_bank" in result.reason


def test_spec_documents_sequential_multi_target_use_case():
    from pathlib import Path

    spec = (Path(__file__).resolve().parents[1] / "spec.md").read_text(
        encoding="utf-8"
    )
    assert "Sequential multi-target sequences" in spec
    assert "--no-reset-to-home" in spec
    assert "IK failure → preparatory repositioning" in spec
    assert "seed bank" in spec.lower() or "seed_bank" in spec
