# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Regression tests: fail-closed planning must not drive colliding GUI motion."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from isaac_sim.target_marker import (
    TARGET_MARKER_COLOR_YELLOW_RGB,
    marker_rgb_for_state,
)
from isaac_sim.viz_plan_policy import (
    MarkerVisualState,
    may_execute_motion,
    plan_result_is_executable,
    resolve_marker_visual_state,
)
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.planning import curobo_planner as cp
from residual_adaptive_ik.planning.curobo_planner import (
    PlannedTrajectory,
    plan_collision_free,
)

REPO = Path(__file__).resolve().parents[1]


def test_may_execute_motion_fail_closed():
    assert may_execute_motion(True) is True
    assert may_execute_motion(False) is False
    assert may_execute_motion(False, gate_on_failure=False) is False


def test_historical_ok_fallback_plan_failed_is_not_executable():
    """The exact bug class: success + ok_fallback wrapping cuRobo plan_failed."""
    bad = PlannedTrajectory(
        waypoints_rad=np.zeros((8, 6)),
        dt_s=1.0 / 60.0,
        success=True,
        backend="numpy_lerp",
        message="ok_fallback|curobo=plan_failed:IK_FAIL",
    )
    assert bad.ok  # dataclass says ok — policy must still refuse execution
    assert plan_result_is_executable(bad) is False


def test_empty_or_failed_plan_not_executable():
    fail = PlannedTrajectory(
        waypoints_rad=np.zeros((0, 6)),
        dt_s=0.02,
        success=False,
        backend="curobo",
        message="rejected_no_numpy_fallback|plan_failed:IK_FAIL",
    )
    assert plan_result_is_executable(fail) is False
    empty_ok = PlannedTrajectory(
        waypoints_rad=np.zeros((0, 6)),
        dt_s=0.02,
        success=True,
        backend="curobo",
        message="ok",
    )
    assert plan_result_is_executable(empty_ok) is False


def test_clean_curobo_success_is_executable():
    good = PlannedTrajectory(
        waypoints_rad=np.linspace(0, 1, 12 * 6).reshape(12, 6),
        dt_s=0.02,
        success=True,
        backend="curobo",
        message="ok",
    )
    assert plan_result_is_executable(good) is True


def test_plan_collision_free_never_returns_unsafe_ok_fallback(monkeypatch):
    """cuRobo reject → not ok, and never ok_fallback|plan_failed."""
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    q0 = 0.5 * (lo + hi)
    q1 = q0.copy()
    q1[0] = float(np.clip(q0[0] + 0.25, lo[0], hi[0]))

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
    assert plan_result_is_executable(traj) is False
    assert "ok_fallback" not in traj.message
    assert "plan_failed" in traj.message
    assert may_execute_motion(plan_result_is_executable(traj)) is False


def test_marker_yellow_when_plan_fails():
    state = resolve_marker_visual_state(plan_ok=False, tip_contacted=False)
    assert state is MarkerVisualState.PLAN_FAIL
    diffuse, _emit = marker_rgb_for_state(state)
    assert diffuse == TARGET_MARKER_COLOR_YELLOW_RGB
    # Contact must not override plan-fail (no green on rejected plan).
    assert (
        resolve_marker_visual_state(plan_ok=False, tip_contacted=True)
        is MarkerVisualState.PLAN_FAIL
    )
    assert (
        resolve_marker_visual_state(plan_ok=True, tip_contacted=True)
        is MarkerVisualState.CONTACT
    )
    assert (
        resolve_marker_visual_state(plan_ok=True, tip_contacted=False)
        is MarkerVisualState.PENDING
    )


def test_viz_script_uses_fail_closed_policy_and_yellow_marker():
    """Source contract: viz must gate on plan_result_is_executable + yellow."""
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "plan_result_is_executable" in src
    assert "may_execute_motion" in src
    assert "MarkerVisualState.PLAN_FAIL" in src
    assert "GATED_NO_MOTION" in src
    assert "_viz_log" in src
    # Must not reintroduce ungated IK lerp to trial.q_sol after plan failure.
    assert "_move_joints_at_hardware_speed(\n                    articulation,\n                    trial.q_sol" not in src
