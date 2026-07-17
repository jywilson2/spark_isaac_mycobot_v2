# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Regression tests: fail-closed planning must not drive colliding GUI motion."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from isaac_sim.target_marker import (
    TARGET_MARKER_COLOR_YELLOW_RGB,
    marker_rgb_for_state,
)
from isaac_sim.viz_plan_policy import (
    MarkerVisualState,
    ViaPressureTracker,
    may_execute_motion,
    meets_min_plan_ok_rate,
    plan_ok_rate,
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


def test_via_pressure_tracker_flags_consecutive_via_episodes():
    """Repeated-via math: EMA of via usage + consecutive-via streak.

    No-via episodes keep pressure low; a sustained run of via episodes drives
    the EMA up and the streak past threshold → ``high`` (systematic bad
    approach), then a clean episode resets the streak.
    """
    t = ViaPressureTracker(alpha=0.5, ema_usage_threshold=0.6, streak_threshold=3)

    # A clean episode (no via) is never "high".
    r = t.update(0)
    assert r["streak"] == 0 and r["ema_usage"] == 0.0 and r["high"] is False

    # First via episode: streak 1, below the streak threshold.
    r = t.update(2)
    assert r["streak"] == 1 and r["ema_usage"] > 0.0 and r["high"] is False

    # Two more consecutive via episodes → streak 3, EMA ≥ 0.6 → high.
    t.update(1)
    r = t.update(3)
    assert r["streak"] == 3
    assert r["ema_usage"] >= 0.6
    assert r["high"] is True
    assert r["max_streak"] >= 3
    assert r["ema_attempts"] > 0.0

    # A clean episode resets the consecutive streak (no longer high).
    r = t.update(0)
    assert r["streak"] == 0 and r["high"] is False
    assert r["max_streak"] == 3  # historical max preserved


def test_via_pressure_tracker_low_pressure_when_vias_sparse():
    """Isolated single-via episodes never trip the streak gate."""
    t = ViaPressureTracker(alpha=0.5, ema_usage_threshold=0.6, streak_threshold=3)
    highs = []
    for a in (1, 0, 1, 0, 1, 0):
        highs.append(t.update(a)["high"])
    assert not any(highs)


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


def test_plan_ok_rate_helpers():
    assert plan_ok_rate(0, 0) == 0.0
    assert plan_ok_rate(2, 46) == pytest.approx(2 / 48)
    assert meets_min_plan_ok_rate(0, 0, min_rate=0.25) is True
    assert meets_min_plan_ok_rate(2, 46, min_rate=0.25) is False
    assert meets_min_plan_ok_rate(12, 36, min_rate=0.25) is True
    assert meets_min_plan_ok_rate(2, 46, min_rate=0.0) is True


def test_viz_script_uses_fail_closed_policy_and_yellow_marker():
    """Source contract: viz must gate on plan_result_is_executable + yellow."""
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "plan_result_is_executable" in src
    assert "may_execute_motion" in src
    assert "MarkerVisualState.PLAN_FAIL" in src
    assert "GATED_NO_MOTION" in src
    assert "_viz_log" in src
    assert "meets_min_plan_ok_rate" in src
    assert "--min-plan-ok-rate" in src
    # CONTACT_HOLD refines to the pierce point on the near surface; must not
    # drive to trial.q_sol (marker center) which reintroduces side/flange
    # immersion.
    assert "CONTACT_HOLD" in src
    assert "axial refine to pierce" in src
    assert "marker\n                # center" in src or "marker center" in src
    fail_idx = src.index("GATED_NO_MOTION")
    # Within ~800 chars after GATED_NO_MOTION there must be no trial.q_sol drive.
    window = src[fail_idx : fail_idx + 800]
    assert "trial.q_sol" not in window


def test_marker_no_contact_reclassified_as_failure():
    """PLAN_OK without tip-face surface contact must count as a failure."""
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "reclassified as PLAN_FAIL" in src
    assert "n_plan_ok -= 1" in src
    assert "n_plan_fail += 1" in src


def test_viz_settle_side_back_immerse_become_plan_fail():
    """Settled side/back/immersed contacts must not stay PLAN_OK / green.

    Source contract: after a green flash, ``classify_tip_contact`` on the
    settled pose must reclassify immersed → ``PLAN_FAIL(immersed)`` and
    side_graze / through / wrong_side_axis → ``PLAN_FAIL(invalid_side)``.
    Side collision-sphere hits at settle also → ``PLAN_FAIL(invalid_side)``
    via ``MARKER_EE_SIDE_SPHERE``. Without this wiring, unit classify rejects
    would not fail the smoke rate.
    """
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "CONTACT_INVALID_SETTLE" in src
    assert "classify_tip_contact" in src
    assert '"immersed"' in src and '"no_contact"' in src and '"invalid_side"' in src
    assert "fail_reason_tag" in src
    assert "PLAN_FAIL({fail_reason_tag})" in src or 'PLAN_FAIL({fail_reason_tag})' in src
    assert "settle_has_side_sphere_hits" in src
    assert "MARKER_EE_SIDE_SPHERE" in src
    # Mid-path diagnostic labels for the same failure modes.
    assert "MARKER_SIDE_GRAZE" in src
    assert "MARKER_WRONG_SIDE" in src
    assert "MARKER_THROUGH" in src or "through" in src
    # Immersion must be an explicit mid-path or settle failure path.
    assert "immersed" in src
    # Mid-path side/back graze latches settle PLAN_FAIL (not warn-only).
    assert "CONTACT_INVALID_MIDPATH_GRAZE" in src
    assert "mid_path_side_or_back" in src
    # Fail-fast after settle reclassify as well as gated planning fails.
    assert "EARLY_ABORT" in src
    assert "early_abort_after_fails" in src


def test_viz_defers_marker_until_plan_outcome():
    """Target sphere must not relocate before planning finishes.

    On PLAN_FAIL the marker may move (yellow) only after recovery timeout;
    on PLAN_OK it moves (red) then the EE follows.
    """
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    # The pre-plan relocate pattern must stay gone.
    assert (
        "target_xyz = np.asarray(trial.target.position_m, dtype=float).reshape(3)\n"
        "            _set_target_marker(stage, target_xyz, state=MarkerVisualState.PENDING)"
        not in src
    )
    # Relocate only when EE motion starts (MARKER_COMMIT) or after timeout (yellow).
    assert "MARKER_COMMIT" in src
    assert "Allowed marker move: planning failed after recovery budget" in src or \
        "marker=yellow after timeout" in src
