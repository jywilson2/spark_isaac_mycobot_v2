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
    ordered_standoff_candidates,
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


def test_ordered_standoff_candidates_near_to_far():
    """Retries progress from near standoffs to farther ones."""
    tip_start = np.array([0.05, 0.0, 0.10])
    center = np.array([0.20, 0.0, 0.10])
    tip_contact = tip_standoff_on_approach(tip_start, center, clearance_m=0.012)
    rows = ordered_standoff_candidates(
        tip_start,
        center,
        tip_contact,
        clearances_m=[0.05, 0.12, 0.18],
        yaws_rad=[0.0],
        radius_m=0.012,
        surface_margin_m=0.015,
        min_travel_m=0.01,
    )
    usable = [r for r in rows if not r[4]]
    assert usable, "expected at least one standoff"
    travels = [r[3] for r in usable]
    assert travels == sorted(travels), "must be nearest → farthest"
    assert travels[0] <= travels[-1]
    # Skips (if any) must appear after usable rows.
    skip_idx = next((i for i, r in enumerate(rows) if r[4]), None)
    if skip_idx is not None:
        assert all(not r[4] for r in rows[:skip_idx])


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


def test_plan_via_standoff_uses_contact_planner_for_via2():
    """via1 uses tip spheres planner; via2 / direct use contact (omit tip)."""

    class _SplitPlanner:
        _interpolation_dt_s = 0.02

        def __init__(self, name: str):
            self.name = name
            self.pose_calls = 0
            self.joint_calls = 0

        def plan_to_joint_goal(self, *args, **kwargs):
            self.joint_calls += 1
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message=f"plan_failed:IK_FAIL|{self.name}",
            )

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            self.pose_calls += 1
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            # Standoff planner (with tip) succeeds via1; contact succeeds via2.
            if self.name == "standoff":
                end = q0 + 0.05
                return PlannedTrajectory(
                    waypoints_rad=np.vstack([q0, end]),
                    dt_s=0.02,
                    success=True,
                    backend="curobo",
                    message="ok|standoff",
                )
            end = q0 + 0.10
            return PlannedTrajectory(
                waypoints_rad=np.vstack([q0, end]),
                dt_s=0.02,
                success=True,
                backend="curobo",
                message="ok|contact",
            )

    standoff = _SplitPlanner("standoff")
    contact = _SplitPlanner("contact")
    q0 = np.zeros(6)
    q1 = np.array([0.2, 0.1, -0.1, 0.0, 0.2, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    traj = plan_via_standoff(
        q0,
        q1,
        planner=standoff,  # type: ignore[arg-type]
        contact_planner=contact,  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05],
        timeout_s=2.0,
    )
    assert traj.ok
    assert contact.joint_calls == 1  # direct on contact planner
    assert standoff.pose_calls == 1  # via1
    assert contact.pose_calls == 1  # via2
    assert "via_standoff" in traj.message


def test_plan_via_standoff_respects_timeout_on_later_clearances():
    """After the mandatory first recovery pass, further work stops when timed out."""

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
    assert "via1_" in traj.message
    assert "recovery_timeout" in traj.message or "timeout" in traj.message
    # One mandatory pass: clearances × default yaws (5) = up to 15 pose calls,
    # but deadline is already past so only the first_pass body runs once.
    assert _AlwaysReject.pose_calls >= 1


def test_plan_via_standoff_executes_partial_via1():
    """via1 OK / via2 FAIL with execute_waypoints moves EE and retries."""

    class _Via1OnlyThenOk:
        _interpolation_dt_s = 0.02

        def __init__(self):
            self.round = 0

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
            self.round += 1
            # First via1 succeeds; first via2 fails; after partial exec, via1+via2 ok.
            if self.round == 1:
                end = q0 + 0.05
                return PlannedTrajectory(
                    waypoints_rad=np.vstack([q0, end]),
                    dt_s=0.02,
                    success=True,
                    backend="curobo",
                    message="ok|via1",
                )
            if self.round == 2:
                return PlannedTrajectory(
                    waypoints_rad=np.zeros((0, 6)),
                    dt_s=0.02,
                    success=False,
                    backend="curobo",
                    message="plan_failed:IK_FAIL",
                )
            end = q0 + 0.02
            return PlannedTrajectory(
                waypoints_rad=np.vstack([q0, end]),
                dt_s=0.02,
                success=True,
                backend="curobo",
                message="ok|retry",
            )

    executed: list[np.ndarray] = []

    def _exec(wp, dt):
        executed.append(np.asarray(wp, dtype=float).copy())

    planner = _Via1OnlyThenOk()
    q0 = np.zeros(6)
    q1 = np.array([0.2, 0.1, -0.1, 0.0, 0.2, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    traj = plan_via_standoff(
        q0,
        q1,
        planner=planner,  # type: ignore[arg-type]
        contact_planner=planner,  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05],
        timeout_s=5.0,
        execute_waypoints=_exec,
    )
    assert traj.ok
    assert len(executed) >= 1
    assert "partial_execs=" in traj.message or "already_executed" in traj.message


def test_collision_yaml_recovery_defaults():
    from residual_adaptive_ik.planning.curobo_planner import load_planning_config

    cfg = load_planning_config()
    assert cfg.get("reset_to_home_before_each_trial", False) is True
    assert cfg.get("plan_recovery_enabled", False) is True
    assert float(cfg.get("plan_recovery_timeout_s", 0)) >= 90.0
    assert int(cfg.get("plan_recovery_direct_max_attempts", 99)) <= 2
    assert len(cfg.get("plan_recovery_standoff_clearances_m", [])) >= 1
    assert float(cfg.get("plan_recovery_min_standoff_travel_m", 0)) >= 0.005
    assert float(cfg.get("plan_recovery_min_standoff_travel_m", 1)) <= 0.05
    assert float(cfg.get("min_plan_ok_rate", 0)) >= 1.0 - 1e-9
    assert cfg.get("plan_recovery_prep_seed_enabled", False) is True
    assert int(cfg.get("plan_recovery_prep_seed_max", 0)) >= 2


def test_try_move_to_preparatory_seed_open_loop_when_plan_fails():
    """INVALID_START-style: planned move fails → open-loop to q_seed."""
    from residual_adaptive_ik.planning.recovery import try_move_to_preparatory_seed
    from residual_adaptive_ik.kinematics.urdf_model import get_default_model

    class _AlwaysInvalidStart:
        _interpolation_dt_s = 0.02

        def plan_to_pose(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION",
            )

        def plan_to_joint_goal(self, *args, **kwargs):
            return self.plan_to_pose()

    executed: list[np.ndarray] = []

    def _exec(wp, dt):
        executed.append(np.asarray(wp, dtype=float).copy())

    q0 = np.array([0.4, -0.2, 0.1, 0.0, 0.2, -0.1])
    q_new, tag, failed = try_move_to_preparatory_seed(
        q0,
        planner=_AlwaysInvalidStart(),  # type: ignore[arg-type]
        model=get_default_model(),
        execute_waypoints=_exec,
        allow_planned=True,
        max_seeds=4,
    )
    assert q_new is not None
    assert "open_loop" in tag
    assert "prep_seed" in tag
    assert len(executed) == 1
    assert executed[0].shape[0] == 2
    assert np.allclose(executed[0][0], q0)
    # Should move toward a bank member (home or mid-home), not stay put.
    assert float(np.linalg.norm(q_new - q0)) > 0.05
    assert len(failed) >= 1


def test_try_move_to_preparatory_seed_prefers_planned_when_ok():
    from residual_adaptive_ik.planning.recovery import try_move_to_preparatory_seed
    from residual_adaptive_ik.kinematics.urdf_model import get_default_model

    class _PlanOk:
        _interpolation_dt_s = 0.02

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            end = q0 + np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
            return PlannedTrajectory(
                waypoints_rad=np.vstack([q0, end]),
                dt_s=0.02,
                success=True,
                backend="curobo",
                message="ok|prep",
            )

    q0 = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])
    q_new, tag, _failed = try_move_to_preparatory_seed(
        q0,
        planner=_PlanOk(),  # type: ignore[arg-type]
        model=get_default_model(),
        allow_planned=True,
        max_seeds=2,
    )
    assert q_new is not None
    assert "planned" in tag
    assert np.allclose(q_new, q0 + np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))


def test_invalid_start_uses_prep_seed_then_falls_through_to_vias():
    """Prep-seed bank then vias — not an infinite home-blend loop."""

    class _InvalidStartThenViaOk:
        _interpolation_dt_s = 0.02

        def __init__(self):
            self.joint_calls = 0
            self.pose_calls = 0

        def plan_to_joint_goal(self, *args, **kwargs):
            self.joint_calls += 1
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION",
            )

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            self.pose_calls += 1
            # Prep-seed helper uses max_attempts=1; via legs use >1.
            if int(max_attempts) <= 1:
                return PlannedTrajectory(
                    waypoints_rad=np.zeros((0, 6)),
                    dt_s=0.02,
                    success=False,
                    backend="curobo",
                    message="plan_failed:MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION",
                )
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            end = q0 + 0.05
            return PlannedTrajectory(
                waypoints_rad=np.vstack([q0, end]),
                dt_s=0.02,
                success=True,
                backend="curobo",
                message="ok|via",
            )

    planner = _InvalidStartThenViaOk()
    q0 = np.array([0.35, -0.15, 0.1, 0.05, 0.1, -0.05])
    q1 = np.array([0.2, 0.1, -0.1, 0.0, 0.2, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    traj = plan_via_standoff(
        q0,
        q1,
        planner=planner,  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05, 0.08, 0.12],
        timeout_s=5.0,
    )
    assert traj.ok, f"Expected OK after via fallthrough, got: {traj.message}"
    assert "via1_" in traj.message or "via_standoff" in traj.message
    # At least one INVALID_START direct + retry after prep-seed / home escape.
    assert planner.joint_calls >= 2
    # Prep uses plan_to_pose(max_attempts=1); vias use max_attempts>1.
    assert planner.pose_calls >= 1


def test_viz_uses_recovery_planner():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "plan_collision_free_with_recovery" in src
    assert "execute_waypoints" in src
    assert "MARKER_COMMIT" in src
