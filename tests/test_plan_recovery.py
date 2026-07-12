# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for standoff via-waypoint planning recovery (no CUDA required)."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.planning.curobo_planner import PlannedTrajectory
from residual_adaptive_ik.planning.recovery import (
    plan_via_standoff,
    tip_standoff_on_approach,
)

REPO = Path(__file__).resolve().parents[1]


def test_tip_standoff_clearance_matches():
    start = np.zeros(3)
    center = np.array([0.25, 0.0, 0.1])
    for c in (0.05, 0.08, 0.12):
        p = tip_standoff_on_approach(start, center, clearance_m=c)
        assert float(np.linalg.norm(p - center)) == pytest.approx(c, abs=1e-9)
        assert p[0] < center[0]


def test_plan_via_standoff_uses_via_when_direct_fails():
    """Fake planner: direct fails; via legs succeed → concatenated path."""

    class _FakePlanner:
        _interpolation_dt_s = 0.02

        def plan_to_joint_goal(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            mid = q0 + 0.05
            end = q0 + 0.10
            return PlannedTrajectory(
                waypoints_rad=np.vstack([q0, mid, end]),
                dt_s=0.02,
                success=True,
                backend="curobo",
                message="ok",
            )

    q0 = np.zeros(6)
    q1 = np.array([0.2, 0.1, -0.1, 0.0, 0.2, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    traj = plan_via_standoff(
        q0,
        q1,
        planner=_FakePlanner(),  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05],
        timeout_s=2.0,
    )
    assert traj.ok
    assert "via_standoff" in traj.message
    assert traj.waypoints_rad.ndim == 2
    assert traj.waypoints_rad.shape[0] >= 4


def test_plan_via_standoff_respects_timeout_on_later_clearances():
    """After the mandatory first via, further clearances stop when timed out."""

    class _AlwaysReject:
        _interpolation_dt_s = 0.02
        pose_calls = 0

        def plan_to_joint_goal(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

        def plan_to_pose(self, *args, **kwargs):
            type(self).pose_calls += 1
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

    _AlwaysReject.pose_calls = 0
    q0 = np.zeros(6)
    q1 = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    traj = plan_via_standoff(
        q0,
        q1,
        planner=_AlwaysReject(),  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05, 0.08, 0.12],
        deadline_monotonic=time.monotonic() - 1.0,
    )
    assert not traj.ok
    # First via still runs; remaining clearances are skipped by timeout.
    assert _AlwaysReject.pose_calls == 1
    assert "via1_" in traj.message


def test_collision_yaml_recovery_defaults():
    from residual_adaptive_ik.planning.curobo_planner import load_planning_config

    cfg = load_planning_config()
    assert cfg.get("reset_to_home_before_each_trial", True) is False
    assert cfg.get("plan_recovery_enabled", False) is True
    assert float(cfg.get("plan_recovery_timeout_s", 0)) >= 15.0
    assert int(cfg.get("plan_recovery_direct_max_attempts", 99)) <= 2
    assert len(cfg.get("plan_recovery_standoff_clearances_m", [])) >= 1


def test_viz_uses_recovery_planner():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "plan_collision_free_with_recovery" in src
