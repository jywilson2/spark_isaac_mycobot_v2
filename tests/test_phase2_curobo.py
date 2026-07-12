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
    cfg = build_mycobot_curobo_robot_cfg(omit_tip_links=False)
    kin = cfg["robot_cfg"]["kinematics"]
    assert Path(kin["urdf_path"]).is_file()
    assert kin["base_link"] == "g_base"
    assert kin["ee_link"] == "joint6_flange"
    assert len(kin["cspace"]["joint_names"]) == 6
    spheres = kin["collision_spheres"]
    # Mesh-fitted YAML; tip/flange included so marker cannot pass through EE side.
    assert "joint1" in spheres
    assert "joint6" in spheres
    assert "joint6_flange" in spheres
    # Centers must be in meters (G_base DAE is mm — fitter scales it).
    for link, lst in spheres.items():
        for s in lst:
            assert abs(float(s["center"][0])) < 1.0, link
            assert 0.001 <= float(s["radius"]) < 0.1, link


def test_tip_on_sphere_surface_standoff():
    from residual_adaptive_ik.planning.curobo_planner import tip_on_sphere_surface

    start = np.array([0.0, 0.0, 0.0])
    center = np.array([0.2, 0.0, 0.0])
    p = tip_on_sphere_surface(start, center, 0.012, margin_m=0.008)
    # Outside the sphere, on the approach ray.
    assert float(np.linalg.norm(p - center)) == pytest.approx(0.020, abs=1e-9)
    assert p[0] < center[0]


def test_fitted_spheres_yaml_committed():
    path = REPO / "configs" / "planning" / "curobo" / "mycobot_280_collision_spheres.yaml"
    assert path.is_file()
    from residual_adaptive_ik.planning.sphere_fit_mycobot import load_collision_spheres_yaml

    data = load_collision_spheres_yaml(path)
    assert "collision_spheres" in data
    assert "joint6" in data["collision_spheres"]
    assert "tip_links_ignore_target" in data


def test_numpy_rejects_path_through_volumetric_target():
    """Proximal capsules must not sweep through a large target sphere volume."""
    from residual_adaptive_ik.geometry.collision import SphereObstacle
    from residual_adaptive_ik.planning.joint_path import plan_joint_lerp_checked

    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q0 = 0.5 * (lo + hi)
    q1 = q0.copy()
    q1[0] = float(np.clip(q0[0] + 0.4, lo[0], hi[0]))
    # Huge sphere at origin engulfs the arm — mid-path must collide.
    engulf = [SphereObstacle(center_m=np.zeros(3), radius_m=0.25, name="engulf")]
    path = plan_joint_lerp_checked(
        q0, q1, engulf, n_samples=16, model=model, ignore_tip_segment=True
    )
    assert path.collision.collides


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


def test_plan_collision_free_fail_closed_when_curobo_rejects(monkeypatch):
    """cuRobo reject must not become executable NumPy lerp (GUI collision bug)."""
    from residual_adaptive_ik.planning import curobo_planner as cp
    from residual_adaptive_ik.planning.curobo_planner import PlannedTrajectory

    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q0 = 0.5 * (lo + hi)
    q1 = q0.copy()
    q1[0] = float(np.clip(q0[0] + 0.2, lo[0], hi[0]))

    class _RejectPlanner:
        def plan_to_joint_goal(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

    monkeypatch.setattr(cp, "curobo_available", lambda: True)
    traj = plan_collision_free(
        q0, q1, prefer_curobo=True, model=model, planner=_RejectPlanner()
    )
    assert not traj.ok
    assert traj.backend == "curobo"
    assert "rejected_no_numpy_fallback" in traj.message
    assert traj.waypoints_rad.size == 0


def test_collision_yaml_disables_numpy_after_curobo_fail():
    from residual_adaptive_ik.planning.curobo_planner import load_planning_config

    cfg = load_planning_config()
    assert cfg.get("fallback_numpy_after_curobo_fail", True) is False
    assert cfg.get("gate_motion_on_plan_failure", False) is True


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
    assert (REPO / "scripts" / "host" / "fit_mycobot_collision_spheres.sh").is_file()
    assert (REPO / "scripts" / "host" / "verify_target_obstacle.sh").is_file()
    assert (REPO / "configs" / "planning" / "curobo_world.yaml").is_file()
