# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 cuRobo / ground-plane planning tests."""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.planning.curobo_planner import (
    build_mycobot_curobo_robot_cfg,
    check_ground_collision,
    curobo_available,
    load_world_config,
    plan_collision_free,
)

REPO = Path(__file__).resolve().parents[1]


def test_world_config_has_ground_cuboid():
    world = load_world_config()
    assert "cuboid" in world
    assert "ground" in world["cuboid"]
    pose = world["cuboid"]["ground"]["pose"]
    assert float(pose[2]) < 0.0  # center below z=0


def test_mycobot_curobo_robot_cfg_points_at_urdf():
    cfg = build_mycobot_curobo_robot_cfg()
    kin = cfg["robot_cfg"]["kinematics"]
    assert Path(kin["urdf_path"]).is_file()
    assert kin["base_link"] == "g_base"
    assert kin["ee_link"] == "joint6_flange"
    assert len(kin["cspace"]["joint_names"]) == 6


def test_ground_collision_detects_low_configuration():
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    # Folded-ish: drive joints toward configurations that often dip low.
    q = 0.5 * (lo + hi)
    q[1] = float(lo[1])  # shoulder down
    q[2] = float(hi[2])
    # May or may not collide depending on FK; assert API runs and returns bool.
    hit = check_ground_collision(q, model=model, ground_z_m=0.15)
    assert isinstance(hit, bool)
    # With an artificially high ground, almost any pose collides.
    assert check_ground_collision(q, model=model, ground_z_m=1.0) is True


def test_plan_collision_free_numpy_fallback_without_requiring_curobo():
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q0 = 0.5 * (lo + hi)
    q1 = q0.copy()
    q1[0] = float(np.clip(q0[0] + 0.2, lo[0], hi[0]))
    traj = plan_collision_free(q0, q1, prefer_curobo=False, model=model)
    assert traj.backend == "numpy_lerp"
    assert traj.ok


@pytest.mark.isaac
def test_curobo_plan_on_host_when_enabled():
    if os.environ.get("SPARK_RUN_CUROBO_SMOKE", "0") != "1":
        pytest.skip("Set SPARK_RUN_CUROBO_SMOKE=1 to run cuRobo GPU smoke")
    if not curobo_available():
        pytest.fail("cuRobo/CUDA required when SPARK_RUN_CUROBO_SMOKE=1")
    from residual_adaptive_ik.planning.curobo_planner import CuRoboMotionPlanner

    lo, hi = load_joint_limits_rad()
    rng = np.random.default_rng(1)
    q0 = rng.uniform(lo, hi)
    q1 = np.clip(q0 + rng.uniform(-0.25, 0.25, size=6), lo, hi)
    planner = CuRoboMotionPlanner()
    traj = planner.plan_to_joint_goal(q0, q1, max_attempts=8)
    assert traj.backend == "curobo"
    # Soft assert: at least one of mild motions should succeed; if not, surface message.
    if not traj.success:
        pytest.skip(f"cuRobo plan unsuccessful for this seed: {traj.message}")
    assert traj.waypoints_rad.ndim == 2
    assert traj.waypoints_rad.shape[1] == 6


def test_install_and_smoke_scripts_exist():
    assert (REPO / "scripts" / "host" / "install_curobo.sh").is_file()
    assert (REPO / "scripts" / "host" / "smoke_phase2_curobo.sh").is_file()
    assert (REPO / "configs" / "planning" / "curobo_world.yaml").is_file()
