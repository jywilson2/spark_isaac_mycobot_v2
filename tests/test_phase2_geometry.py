# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 geometry + joint-path collision tests (CI, no Isaac Kit)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from residual_adaptive_ik.geometry import (
    SphereObstacle,
    capsule_sphere_collide,
    check_config_collision,
    link_capsules_from_q,
)
from residual_adaptive_ik.geometry.collision import Capsule
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.kinematics.validation import load_validation_config, validate_solution
from residual_adaptive_ik.planning import interpolate_joint_path, plan_joint_lerp_checked

REPO = Path(__file__).resolve().parents[1]
COLLISION_YAML = REPO / "configs" / "planning" / "collision.yaml"


def test_collision_config_loads():
    raw = yaml.safe_load(COLLISION_YAML.read_text(encoding="utf-8"))
    assert float(raw["link_radius_m"]) > 0.0
    assert int(raw["path_samples"]) >= 1


def test_capsule_sphere_known_intersection():
    cap = Capsule(a_m=[0.0, 0.0, 0.0], b_m=[0.1, 0.0, 0.0], radius_m=0.01)
    hit = SphereObstacle(center_m=[0.05, 0.0, 0.0], radius_m=0.01, name="mid")
    miss = SphereObstacle(center_m=[0.05, 0.5, 0.0], radius_m=0.01, name="far")
    assert capsule_sphere_collide(cap, hit)
    assert not capsule_sphere_collide(cap, miss)


def test_link_capsules_have_six_segments_for_mycobot():
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q = 0.5 * (lo + hi)
    caps = link_capsules_from_q(q, model=model, link_radius_m=0.025)
    # 6 revolute frames + EE → 6 segments
    assert len(caps) == 6


def test_interpolate_joint_path_endpoints():
    q0 = np.zeros(6)
    q1 = np.ones(6)
    path = interpolate_joint_path(q0, q1, n_samples=10)
    assert path.shape == (11, 6)
    np.testing.assert_allclose(path[0], q0)
    np.testing.assert_allclose(path[-1], q1)


def test_plan_detects_proximal_sweep_through_nearby_sphere():
    """A fat obstacle on the base can collide mid-path even if tip goal is clear."""
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q_start = 0.5 * (lo + hi)
    q_goal = q_start.copy()
    q_goal[0] = float(np.clip(q_start[0] + 1.0, lo[0], hi[0]))
    # Huge sphere centered at origin — must collide with proximal capsules.
    obs = [SphereObstacle(center_m=[0.0, 0.0, 0.12], radius_m=0.08, name="base_blob")]
    path = plan_joint_lerp_checked(
        q_start,
        q_goal,
        obs,
        n_samples=16,
        model=model,
        link_radius_m=0.03,
        ignore_tip_segment=True,
    )
    assert path.collision.collides
    assert len(path.collision.reasons) >= 1


def test_validate_solution_geometry_obstacles():
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q = 0.5 * (lo + hi)
    pose = forward_kinematics(q, model=model)
    cfg = load_validation_config()
    cfg = dict(cfg)
    cfg["enforce_collision"] = True
    # Obstacle far away — should still pass FK checks.
    far = [SphereObstacle(center_m=[2.0, 2.0, 2.0], radius_m=0.05, name="far")]
    ok = validate_solution(
        q,
        pose,
        cfg,
        q_ik=q,
        residual_q=np.zeros(6),
        model=model,
        joint_lower_rad=lo,
        joint_upper_rad=hi,
        obstacles=far,
        ignore_tip_segment=True,
    )
    assert ok.ok
    # Obstacle engulfing the arm — must fail with collision.
    engulf = [SphereObstacle(center_m=[0.0, 0.0, 0.15], radius_m=0.25, name="engulf")]
    bad = validate_solution(
        q,
        pose,
        cfg,
        q_ik=q,
        residual_q=np.zeros(6),
        model=model,
        joint_lower_rad=lo,
        joint_upper_rad=hi,
        obstacles=engulf,
        ignore_tip_segment=False,
    )
    assert not bad.ok
    assert "collision" in bad.reasons


def test_phase_renumber_scripts_exist():
    assert (REPO / "scripts" / "run_phase2_geometry.sh").is_file()
    assert (REPO / "scripts" / "run_phase3_supervised.sh").is_file()
    assert (REPO / "scripts" / "run_phase4_sac.sh").is_file()
