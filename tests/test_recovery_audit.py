# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Headless tests: detect PLAN_FAIL without via-recovery attempts."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.planning.curobo_planner import PlannedTrajectory
from residual_adaptive_ik.planning.recovery import (
    plan_via_standoff,
    recovery_audit_plan_fail,
    recovery_via_attempts_in_message,
)

REPO = Path(__file__).resolve().parents[1]


def test_audit_flags_plan_fail_without_via():
    """The GUI bug class: yellow marker, no via1_ in the plan message."""
    bad = "rejected_no_numpy_fallback|plan_failed:IK_FAIL"
    audit = recovery_audit_plan_fail(bad, recovery_enabled=True, had_marker=True)
    assert audit["ok"] is False
    assert audit["reason"] == "no_via_attempt_after_direct_fail"


def test_audit_passes_when_vias_logged():
    msg = (
        "recovery_exhausted|via_attempts=2|direct:plan_failed:IK_FAIL|"
        "via1_0.050:plan_failed:IK_FAIL|via1_0.080:plan_failed:IK_FAIL"
    )
    audit = recovery_audit_plan_fail(msg, recovery_enabled=True, had_marker=True)
    assert audit["ok"] is True
    assert recovery_via_attempts_in_message(msg) == 2


def test_plan_via_always_attempts_via_even_if_direct_burns_deadline():
    """Direct call may finish after the soft deadline; first via must still run."""

    class _SlowDirectThenRejectVias:
        _interpolation_dt_s = 0.02
        pose_calls = 0

        def plan_to_joint_goal(self, *args, **kwargs):
            # Simulate a long cuRobo direct failure past the soft deadline.
            time.sleep(0.05)
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

    _SlowDirectThenRejectVias.pose_calls = 0
    q0 = np.zeros(6)
    q1 = np.array([0.2, 0.1, -0.1, 0.0, 0.2, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    # Soft deadline already in the past when vias would be gated by a naive check.
    traj = plan_via_standoff(
        q0,
        q1,
        planner=_SlowDirectThenRejectVias(),  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05, 0.08],
        deadline_monotonic=time.monotonic() - 1.0,
    )
    assert not traj.ok
    assert _SlowDirectThenRejectVias.pose_calls >= 1, (
        "expected at least one via plan_to_pose after direct fail"
    )
    assert recovery_via_attempts_in_message(traj.message) >= 1
    audit = recovery_audit_plan_fail(traj.message, recovery_enabled=True, had_marker=True)
    assert audit["ok"] is True


def test_diagnose_plan_recovery_script_exists():
    path = REPO / "scripts" / "host" / "diagnose_plan_recovery.sh"
    assert path.is_file()
    text = path.read_text(encoding="utf-8")
    assert "recovery_audit_plan_fail" in text
    assert "via_attempts" in text


def test_viz_logs_recovery_audit_fields():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "recovery_via_attempts_in_message" in src or "via_attempts" in src
    assert "RECOVERY" in src or "via_attempts" in src
