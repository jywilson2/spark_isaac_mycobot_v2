# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Tests for the Pinocchio dexterity-gate IK backend (skip if unavailable)."""
from __future__ import annotations

import numpy as np
import pytest

from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.planning.dexterity import (
    contact_pose_is_dexterous,
    resolve_prescreen_backend,
)
from residual_adaptive_ik.planning.pinocchio_ik import pinocchio_available


@pytest.mark.skipif(not pinocchio_available(), reason="pinocchio not installed")
def test_pinocchio_solves_identity_fk_pose() -> None:
    from residual_adaptive_ik.planning.pinocchio_ik import solve_pose_ik

    lo, hi = load_joint_limits_rad()
    q = 0.5 * (lo + hi)
    pose = forward_kinematics(q)
    res = solve_pose_ik(
        pose.position_m,
        pose.quaternion_wxyz,
        seed_q=q,
        position_tol_m=1e-3,
        orientation_tol_rad=1e-2,
    )
    assert res.success, (res.reason, res.position_error_m, res.orientation_error_rad)


@pytest.mark.skipif(not pinocchio_available(), reason="pinocchio not installed")
def test_pinocchio_backend_selected_by_auto() -> None:
    assert resolve_prescreen_backend("auto") == "pinocchio"


@pytest.mark.skipif(not pinocchio_available(), reason="pinocchio not installed")
def test_pinocchio_prescreen_skips_far_target() -> None:
    res = contact_pose_is_dexterous(
        [0.6, 0.0, 0.1], 0.02, n_seeds=8, backend="pinocchio"
    )
    assert not res.feasible
    assert res.classification == "outside_dexterous_region"
    assert res.backend == "pinocchio"


def test_numpy_backend_forced_without_pinocchio() -> None:
    assert resolve_prescreen_backend("numpy") == "numpy"
    res = contact_pose_is_dexterous(
        [0.6, 0.0, 0.1], 0.02, n_seeds=4, backend="numpy"
    )
    assert not res.feasible
    assert res.backend == "numpy"
