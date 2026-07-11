# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""IK validation + DLS solver contract tests — Phase 1 acceptance."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import (
    DampedLeastSquaresIK,
    load_joint_limits_rad,
)
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.kinematics.validation import (
    load_validation_config,
    validate_solution,
)
from residual_adaptive_ik.utils.math_utils import DEG2RAD
from residual_adaptive_ik.utils.transforms import orientation_error_magnitude_rad

REPO = Path(__file__).resolve().parents[1]
KINEMATICS_URDF = REPO / "assets" / "urdf" / "mycobot_280_m5_kinematics.urdf"
VALIDATION_YAML = REPO / "configs" / "ik" / "validation.yaml"


@pytest.fixture(scope="module")
def model():
    return load_urdf_model(KINEMATICS_URDF)


@pytest.fixture(scope="module")
def val_cfg():
    return load_validation_config(VALIDATION_YAML)


@pytest.fixture(scope="module")
def joint_limits():
    return load_joint_limits_rad(REPO / "configs" / "robot" / "joint_limits.yaml")


def test_validation_config_loads():
    cfg = load_validation_config(VALIDATION_YAML)
    assert cfg["max_position_error_m"] == 0.001
    assert cfg["max_residual_deg"] == 0.5
    assert cfg["workspace_radius_m"] == 0.280


def test_validate_rejects_bad_shape(val_cfg):
    pose = Pose(position_m=np.zeros(3), quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]))
    result = validate_solution(np.zeros(5), pose, val_cfg)
    assert not result.ok
    assert any(r.startswith("bad_shape") for r in result.reasons)


def test_validate_rejects_nan(val_cfg):
    pose = Pose(position_m=np.zeros(3), quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]))
    q = np.zeros(6)
    q[0] = np.nan
    result = validate_solution(q, pose, val_cfg)
    assert not result.ok
    assert "nan_in_q" in result.reasons


def test_validate_rejects_joint_limit(val_cfg, joint_limits, model):
    lo, hi = joint_limits
    q = 0.5 * (lo + hi)
    q[0] = hi[0] + 0.5
    pose = forward_kinematics(0.5 * (lo + hi), model=model)
    result = validate_solution(
        q, pose, val_cfg, model=model, joint_lower_rad=lo, joint_upper_rad=hi
    )
    assert not result.ok
    assert "joint_limit_violation" in result.reasons


def test_validate_rejects_oversized_residual(val_cfg, joint_limits, model):
    lo, hi = joint_limits
    q_ik = 0.5 * (lo + hi)
    pose = forward_kinematics(q_ik, model=model)
    # Disable FK/workspace checks by using matching pose; residual still fails.
    residual = np.full(6, 2.0 * DEG2RAD)  # 2° > 0.5° default
    q = q_ik + residual
    # Keep q inside limits for a clean residual-only failure
    q = np.clip(q, lo, hi)
    residual = q - q_ik
    cfg = dict(val_cfg)
    cfg["enforce_workspace_radius"] = False
    result = validate_solution(
        q,
        pose,
        cfg,
        q_ik=q_ik,
        residual_q=residual,
        model=model,
        joint_lower_rad=lo,
        joint_upper_rad=hi,
    )
    assert not result.ok
    assert "residual_bound_exceeded" in result.reasons


def test_validate_accepts_true_fk_pose_in_workspace(val_cfg, joint_limits, model):
    lo, hi = joint_limits
    rng = np.random.default_rng(1)
    # Find a configuration whose EE is inside the workspace ball
    q = None
    pose = None
    for _ in range(200):
        candidate = rng.uniform(lo, hi)
        p = forward_kinematics(candidate, model=model)
        if float(np.linalg.norm(p.position_m)) <= float(val_cfg["workspace_radius_m"]):
            q = candidate
            pose = p
            break
    assert q is not None and pose is not None
    result = validate_solution(
        q,
        pose,
        val_cfg,
        q_ik=q,
        residual_q=np.zeros(6),
        model=model,
        joint_lower_rad=lo,
        joint_upper_rad=hi,
    )
    assert result.ok, result.reasons


def test_dls_recovers_reachable_pose(model, joint_limits):
    lo, hi = joint_limits
    rng = np.random.default_rng(2)
    solver = DampedLeastSquaresIK.from_config(model=model)
    successes = 0
    trials = 20
    for _ in range(trials):
        q_true = rng.uniform(lo, hi)
        target = forward_kinematics(q_true, model=model)
        if float(np.linalg.norm(target.position_m)) > 0.28:
            continue
        seed = np.clip(q_true + rng.uniform(-0.1, 0.1, size=6), lo, hi)
        result = solver.solve(target, seed_q=seed)
        if result.success:
            successes += 1
            assert result.position_error_m <= solver.position_tol_m + 1e-9
            assert result.orientation_error_rad <= solver.orientation_tol_rad + 1e-9
            assert np.all(np.isfinite(result.q))
    assert successes >= 5, f"expected several successes, got {successes}"


def test_dls_never_marks_nonfinite_success(model):
    solver = DampedLeastSquaresIK.from_config(model=model)
    bad = Pose(
        position_m=np.array([np.nan, 0.0, 0.0]),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    result = solver.solve(bad, seed_q=np.zeros(6))
    assert not result.success


def test_dls_reports_failure_reason_on_unreachable(model, joint_limits):
    """Far-away target should fail without claiming success."""
    lo, hi = joint_limits
    solver = DampedLeastSquaresIK(
        model=model,
        max_iterations=30,
        position_tol_m=1e-3,
        orientation_tol_rad=1e-2,
        joint_lower_rad=lo,
        joint_upper_rad=hi,
    )
    target = Pose(
        position_m=np.array([2.0, 2.0, 2.0]),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    result = solver.solve(target, seed_q=0.5 * (lo + hi))
    assert not result.success
    assert result.reason
    assert np.all(np.isfinite(result.q))


def test_jacobian_matches_finite_difference(model, joint_limits):
    lo, hi = joint_limits
    q = 0.5 * (lo + hi)
    J = model.geometric_jacobian(q)
    assert J.shape == (6, 6)
    eps = 1e-7
    pose0 = forward_kinematics(q, model=model)
    J_fd = np.zeros((6, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        pose1 = forward_kinematics(q + dq, model=model)
        J_fd[:3, i] = (pose1.position_m - pose0.position_m) / eps
        # Angular FD via orientation error / eps
        from residual_adaptive_ik.utils.transforms import orientation_error_rad

        J_fd[3:, i] = orientation_error_rad(pose0.quaternion_wxyz, pose1.quaternion_wxyz) / eps
    assert np.allclose(J[:3], J_fd[:3], atol=1e-5)
    assert np.allclose(J[3:], J_fd[3:], atol=1e-4)


def test_baseline_eval_small(model):
    from residual_adaptive_ik.kinematics.baseline_eval import evaluate_baseline

    metrics = evaluate_baseline(n_poses=50, seed=0, urdf_path=KINEMATICS_URDF)
    assert metrics["n_poses"] == 50
    assert 0.0 <= metrics["success_rate"] <= 1.0
    assert metrics["n_success"] + metrics["n_failure"] == 50
    # With workspace filtering and mild seed jitter, expect a solid majority.
    assert metrics["success_rate"] >= 0.7
