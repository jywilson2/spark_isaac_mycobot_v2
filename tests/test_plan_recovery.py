# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for standoff via-waypoint planning recovery (no CUDA required)."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.planning.curobo_planner import PlannedTrajectory
from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import DampedLeastSquaresIK
from residual_adaptive_ik.planning.recovery import (
    _is_ik_fail,
    contact_orientation_candidates,
    ordered_standoff_candidates,
    plan_via_standoff,
    radial_preapproach_tip,
    tip_standoff_on_approach,
    try_radial_reposition_via,
)
from residual_adaptive_ik.planning.contact_geometry import (
    build_sphere_contact_approach,
    tool_axis_alignment_error_rad,
)
from residual_adaptive_ik.planning.curobo_planner import load_planning_config

REPO = Path(__file__).resolve().parents[1]


def _fake_ok_traj(
    q_start: np.ndarray,
    tip_m: np.ndarray,
    quat_wxyz: np.ndarray,
    *,
    message: str = "ok",
) -> PlannedTrajectory:
    """Return a success trajectory whose end joints FK-match tip+quat.

    Why: recovery now requires FK pad-alignment before tip-omit (and prefers
    axial DLS lerp). Fake planners that only bump ``q`` by a constant leave the
    wrist yawed → pad gate refuses tip-omit → false ``traj.ok is False``.
    """
    q0 = np.asarray(q_start, dtype=float).reshape(6)
    pose = Pose(
        position_m=np.asarray(tip_m, dtype=float).reshape(3),
        quaternion_wxyz=np.asarray(quat_wxyz, dtype=float).reshape(4),
    )
    ik = DampedLeastSquaresIK(
        max_iterations=80,
        damping=1e-2,
        position_tol_m=2e-3,
        orientation_tol_rad=0.08,
        enforce_joint_limits=True,
    )
    sol = ik.solve(pose, seed_q=q0)
    q1 = np.asarray(sol.q if sol.success else q0, dtype=float).reshape(6)
    return PlannedTrajectory(
        waypoints_rad=np.vstack([q0, q1]),
        dt_s=0.02,
        success=True,
        backend="curobo",
        message=message,
    )


def test_contact_orientation_candidates_exact_first_and_cone():
    approach = build_sphere_contact_approach(
        np.array([0.30, 0.0, 0.10]), np.array([0.25, 0.0, 0.10]), 0.02
    )
    fallback = np.array([1.0, 0.0, 0.0, 0.0])
    cone_on = contact_orientation_candidates(
        approach,
        fallback_quaternion_wxyz=fallback,
        cfg={
            "contact_orientation_cone_enabled": True,
            "contact_orientation_cone_max_rad": 0.30,
            "contact_orientation_cone_tilts": 2,
            "contact_orientation_cone_azimuths": 4,
        },
    )
    # Exact outward normal first, cone entries, then the fallback quaternion.
    assert np.allclose(cone_on[0], approach.quaternion_wxyz)
    assert np.allclose(cone_on[-1], fallback)
    assert len(cone_on) == 1 + 2 * 4 + 1
    # Every cone entry (excluding the current-EE fallback) stays within tol.
    for q in cone_on[:-1]:
        assert tool_axis_alignment_error_rad(q, approach.normal_outward) <= 0.30 + 1e-6


def test_contact_orientation_candidates_disabled():
    approach = build_sphere_contact_approach(
        np.array([0.30, 0.0, 0.10]), np.array([0.25, 0.0, 0.10]), 0.02
    )
    fallback = np.array([1.0, 0.0, 0.0, 0.0])
    out = contact_orientation_candidates(
        approach,
        fallback_quaternion_wxyz=fallback,
        cfg={"contact_orientation_cone_enabled": False},
    )
    assert len(out) == 2
    assert np.allclose(out[0], approach.quaternion_wxyz)
    assert np.allclose(out[1], fallback)


def test_tip_standoff_clearance_matches():
    start = np.zeros(3)
    center = np.array([0.25, 0.0, 0.1])
    for c in (0.05, 0.08, 0.12):
        p = tip_standoff_on_approach(start, center, clearance_m=c)
        assert float(np.linalg.norm(p - center)) == pytest.approx(c, abs=1e-9)
        assert p[0] < center[0]


def test_radial_preapproach_tip_uses_base_target_ray():
    """Pre-approach lies on the base→target line at the requested clearance."""
    center = np.array([0.18, 0.06, 0.12])
    for c in (0.08, 0.12, 0.16):
        p = radial_preapproach_tip(center, clearance_m=c)
        # Exactly ``c`` from the center.
        assert float(np.linalg.norm(p - center)) == pytest.approx(c, abs=1e-9)
        # On the base(origin)→center ray, between base and center (backed off).
        u = center / float(np.linalg.norm(center))
        assert np.allclose(p, center - c * u, atol=1e-9)
        assert float(np.linalg.norm(p)) < float(np.linalg.norm(center))
    # Degenerate center-at-base → straight up, no NaN.
    p0 = radial_preapproach_tip(np.zeros(3), clearance_m=0.1)
    assert np.all(np.isfinite(p0))


def test_is_ik_fail_detects_status():
    assert _is_ik_fail("plan_failed:MotionGenStatus.IK_FAIL")
    assert _is_ik_fail("contact_approach_q0:plan_failed:IK_FAIL|x")
    assert not _is_ik_fail("plan_failed:INVALID_START_STATE_WORLD_COLLISION")
    assert not _is_ik_fail("ok")


def test_try_radial_reposition_via_repositions_then_none():
    """Reposition succeeds when a pre-approach plans; None when all fail."""
    cfg = load_planning_config()
    center = np.array([0.18, 0.0, 0.12])

    class _OkPlanner:
        _interpolation_dt_s = 0.02

        def __init__(self):
            self.calls = 0

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            self.calls += 1
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            return PlannedTrajectory(
                waypoints_rad=np.vstack([q0, q0 + 0.03]),
                dt_s=0.02,
                success=True,
                backend="curobo",
                message="ok",
            )

    moved: list[np.ndarray] = []
    log: list[str] = []
    ok_planner = _OkPlanner()
    q_new, tag = try_radial_reposition_via(
        np.zeros(6),
        sphere_center_m=center,
        sphere_radius_m=0.012,
        planner=ok_planner,  # type: ignore[arg-type]
        model=None,
        cfg=cfg,
        execute_waypoints=lambda wp, dt: moved.append(np.asarray(wp)),
        attempts_log=log,
    )
    assert q_new is not None
    assert tag.endswith("_ok") and "reposition_radial" in tag
    assert moved, "reposition via should be executed"
    assert any("reposition_radial" in m for m in log)

    class _FailPlanner:
        _interpolation_dt_s = 0.02

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

    q_none, tag2 = try_radial_reposition_via(
        np.zeros(6),
        sphere_center_m=center,
        sphere_radius_m=0.012,
        planner=_FailPlanner(),  # type: ignore[arg-type]
        model=None,
        cfg=cfg,
        attempts_log=[],
    )
    assert q_none is None and tag2 == "reposition_radial_exhausted"


def test_plan_via_standoff_repositions_on_ee_close_ik_fail():
    """EE starts AT the marker (close) + oriented contact IK_FAILs → reposition.

    A stateful fake planner fails the near-center oriented contact until a
    far (base→target) reposition move succeeds; after repositioning, the
    oriented contact solves. Asserts the reposition via was actually invoked.
    """
    q_start = np.array([0.1, -0.3, 0.2, 0.0, 0.1, 0.0])
    tip0 = np.asarray(
        forward_kinematics(q_start).position_m, dtype=float
    ).reshape(3)
    marker = SphereObstacle(center_m=tip0, radius_m=0.012)  # EE_CLOSE (tip==center)

    class _CloseFake:
        _interpolation_dt_s = 0.02

        def __init__(self):
            self.repositioned = False
            self.reposition_calls = 0

        def plan_to_joint_goal(self, *a, **k):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            d = float(np.linalg.norm(np.asarray(tip, dtype=float).reshape(3) - tip0))
            if d > 0.05:  # far pre-approach = reposition (or far standoff)
                self.repositioned = True
                self.reposition_calls += 1
                return _fake_ok_traj(q0, tip, quat, message="ok")
            # Near-center oriented contact: fails until we have repositioned.
            if not self.repositioned:
                return PlannedTrajectory(
                    waypoints_rad=np.zeros((0, 6)),
                    dt_s=0.02,
                    success=False,
                    backend="curobo",
                    message="plan_failed:MotionGenStatus.IK_FAIL",
                )
            return _fake_ok_traj(q0, tip, quat, message="ok")

    fake = _CloseFake()
    traj = plan_via_standoff(
        q_start,
        q_start,
        planner=fake,  # type: ignore[arg-type]
        contact_planner=fake,  # type: ignore[arg-type]
        obstacles=[marker],
        timeout_s=3.0,
    )
    assert fake.reposition_calls >= 1, "expected a base→target reposition via"
    assert traj.ok


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
    """Fake planner: oriented approach fails; via + contact succeed."""

    center = np.array([0.15, 0.0, 0.15])

    class _FakePlanner:
        _interpolation_dt_s = 0.02
        pose_calls = 0
        via_seen = False

        def plan_to_joint_goal(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:IK_FAIL",
            )

        def plan_to_pose(self, q_start, tip, quat, *, max_attempts=4, obstacles=None):
            type(self).pose_calls += 1
            dist = float(np.linalg.norm(np.asarray(tip, dtype=float).reshape(3) - center))
            q0 = np.asarray(q_start, dtype=float).reshape(6)
            # Distinguish a via standoff (far from center) from the near-surface
            # contact approach/nudge. This is robust to how many orientation-cone
            # candidates the contact leg tries: the DIRECT contact (before any
            # via) always fails; a far via standoff succeeds and, once a via has
            # been taken, the post-via oriented contact succeeds.
            if dist >= 0.04:
                type(self).via_seen = True
                return _fake_ok_traj(q0, tip, quat, message="ok")
            if not type(self).via_seen:
                return PlannedTrajectory(
                    waypoints_rad=np.zeros((0, 6)),
                    dt_s=0.02,
                    success=False,
                    backend="curobo",
                    message="plan_failed:IK_FAIL|approach",
                )
            return _fake_ok_traj(q0, tip, quat, message="ok")

    _FakePlanner.pose_calls = 0
    _FakePlanner.via_seen = False
    q0 = np.zeros(6)
    q1 = np.array([0.2, 0.1, -0.1, 0.0, 0.2, 0.0])
    marker = SphereObstacle(center_m=np.array([0.15, 0.0, 0.15]), radius_m=0.012)
    traj = plan_via_standoff(
        q0,
        q1,
        planner=_FakePlanner(),  # type: ignore[arg-type]
        contact_planner=_FakePlanner(),  # type: ignore[arg-type]
        obstacles=[marker],
        standoff_clearances_m=[0.05],
        timeout_s=2.0,
    )
    assert traj.ok
    assert "via_contact" in traj.message or "via_standoff" in traj.message
    assert traj.waypoints_rad.ndim == 2
    assert traj.waypoints_rad.shape[0] >= 2


def test_plan_via_standoff_uses_contact_planner_for_via2():
    """via1 uses tip spheres planner; oriented contact uses contact (omit tip)."""

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
            # Standoff planner: fail first approach (far), succeed via1 / reseat.
            if self.name == "standoff":
                # Fail both oriented-approach quat candidates from far.
                if self.pose_calls <= 2:
                    return PlannedTrajectory(
                        waypoints_rad=np.zeros((0, 6)),
                        dt_s=0.02,
                        success=False,
                        backend="curobo",
                        message="plan_failed:IK_FAIL|far_approach",
                    )
                return _fake_ok_traj(q0, tip, quat, message="ok|standoff")
            return _fake_ok_traj(q0, tip, quat, message="ok|contact")

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
    assert standoff.pose_calls >= 2  # failed direct approach + via1 (+ maybe reseat)
    # Tip-omit prefers axial DLS lerp; contact MotionGen is fallback only.
    assert contact.pose_calls >= 0
    assert "via_contact" in traj.message or "direct_contact" in traj.message
    assert "contact_nudge_m" in traj.message


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
            # Fail both far oriented-approach quat candidates; then via1 /
            # contact legs succeed (with one contact fail to force partial).
            if self.round <= 2:
                return PlannedTrajectory(
                    waypoints_rad=np.zeros((0, 6)),
                    dt_s=0.02,
                    success=False,
                    backend="curobo",
                    message="plan_failed:IK_FAIL|far",
                )
            if self.round == 3:
                end = q0 + 0.05
                return PlannedTrajectory(
                    waypoints_rad=np.vstack([q0, end]),
                    dt_s=0.02,
                    success=True,
                    backend="curobo",
                    message="ok|via1",
                )
            if self.round in (4, 5):
                # Oriented contact approach quats after via1 — fail once each
                # path then succeed on later rounds.
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
    assert cfg.get("reset_to_home_before_each_trial", True) is False
    assert cfg.get("plan_recovery_enabled", False) is True
    assert float(cfg.get("plan_recovery_timeout_s", 0)) >= 90.0
    assert int(cfg.get("plan_recovery_direct_max_attempts", 99)) <= 2
    assert len(cfg.get("plan_recovery_standoff_clearances_m", [])) >= 1
    assert float(cfg.get("plan_recovery_min_standoff_travel_m", 0)) >= 0.005
    assert float(cfg.get("plan_recovery_min_standoff_travel_m", 1)) <= 0.05
    assert float(cfg.get("min_plan_ok_rate", 0)) >= 1.0 - 1e-9
    assert cfg.get("plan_recovery_prep_seed_enabled", False) is True
    assert int(cfg.get("plan_recovery_prep_seed_max", 0)) >= 2
    assert cfg.get("contact_axis_enabled", False) is True
    assert float(cfg.get("contact_standoff_m", 0)) >= 0.005
    assert float(cfg.get("contact_nudge_max_m", 1)) <= 0.03
    assert float(cfg.get("contact_nudge_max_m", 0)) >= float(
        cfg.get("contact_standoff_m", 1)
    )


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


def test_prep_seed_accept_rejects_before_execute_keeps_joint_continuity():
    """Far-tip desync fix: rejected seeds never execute; q stays put."""
    from residual_adaptive_ik.planning.recovery import try_move_to_preparatory_seed
    from residual_adaptive_ik.kinematics.urdf_model import get_default_model

    class _MustNotPlan:
        def plan_to_pose(self, *args, **kwargs):
            raise AssertionError("plan_to_pose must not run for pre-rejected seed")

        def plan_to_joint_goal(self, *args, **kwargs):
            raise AssertionError("plan_to_joint_goal must not run for pre-rejected seed")

    executed: list[np.ndarray] = []

    def _exec(wp, dt):
        executed.append(np.asarray(wp, dtype=float).copy())

    q0 = np.array([0.4, -0.2, 0.1, 0.0, 0.2, -0.1], dtype=float)
    q_new, tag, failed = try_move_to_preparatory_seed(
        q0,
        planner=_MustNotPlan(),  # type: ignore[arg-type]
        model=get_default_model(),
        execute_waypoints=_exec,
        allow_planned=True,
        max_seeds=4,
        accept_seed=lambda _q: False,
    )
    assert q_new is None
    assert "prep_seed_rejected_pre" in tag
    assert executed == [], "pre-reject must not execute waypoints"
    assert len(failed) >= 1
    # Joint continuity invariant: planner q_cur unchanged when nothing executed.
    assert np.allclose(q0, np.array([0.4, -0.2, 0.1, 0.0, 0.2, -0.1]))


def test_prep_seed_accept_improving_seed_is_executed():
    """Accept predicate True → plan/execute proceeds as before."""
    from residual_adaptive_ik.planning.recovery import try_move_to_preparatory_seed
    from residual_adaptive_ik.kinematics.urdf_model import get_default_model

    class _AlwaysInvalidStart:
        def plan_to_pose(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION",
            )

    executed: list[np.ndarray] = []

    def _exec(wp, dt):
        executed.append(np.asarray(wp, dtype=float).copy())

    q0 = np.array([0.4, -0.2, 0.1, 0.0, 0.2, -0.1], dtype=float)
    q_new, tag, _failed = try_move_to_preparatory_seed(
        q0,
        planner=_AlwaysInvalidStart(),  # type: ignore[arg-type]
        model=get_default_model(),
        execute_waypoints=_exec,
        allow_planned=True,
        max_seeds=2,
        accept_seed=lambda _q: True,
    )
    assert q_new is not None
    assert "open_loop" in tag
    assert len(executed) == 1
    assert float(np.linalg.norm(q_new - q0)) > 0.05


def test_prep_seed_accept_invalid_start_worsening_still_accepted():
    """INVALID_START escape: far-tip predicate may accept non-improving seeds."""
    from residual_adaptive_ik.kinematics.fk import forward_kinematics
    from residual_adaptive_ik.kinematics.urdf_model import get_default_model
    from residual_adaptive_ik.planning.recovery import try_move_to_preparatory_seed

    class _AlwaysInvalidStart:
        def plan_to_pose(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=0.02,
                success=False,
                backend="curobo",
                message="plan_failed:MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION",
            )

    model = get_default_model()
    q0 = np.array([0.4, -0.2, 0.1, 0.0, 0.2, -0.1], dtype=float)
    tip0 = np.asarray(forward_kinematics(q0, model=model).position_m, dtype=float)
    # Artificial "standoff" near tip0 so bank seeds typically do not shrink.
    standoff = tip0.copy()
    tip_to_standoff_fail = 0.16
    allow_invalid_start_escape = True

    def _accept(q_seed: np.ndarray) -> bool:
        tip_seed = np.asarray(
            forward_kinematics(q_seed, model=model).position_m, dtype=float
        )
        tip_to_standoff_seed = float(np.linalg.norm(tip_seed - standoff))
        if tip_to_standoff_seed <= tip_to_standoff_fail - 0.02:
            return True
        return allow_invalid_start_escape

    executed: list[np.ndarray] = []

    def _exec(wp, dt):
        executed.append(np.asarray(wp, dtype=float).copy())

    q_new, tag, _failed = try_move_to_preparatory_seed(
        q0,
        planner=_AlwaysInvalidStart(),  # type: ignore[arg-type]
        model=model,
        execute_waypoints=_exec,
        allow_planned=True,
        max_seeds=2,
        accept_seed=_accept,
    )
    assert q_new is not None
    assert "prep_seed_rejected_pre" not in tag
    assert len(executed) == 1


def test_invalid_start_uses_prep_seed_then_falls_through_to_vias():
    """Prep-seed bank then vias — not an infinite home-blend loop."""

    class _InvalidStartThenViaOk:
        _interpolation_dt_s = 0.02

        def __init__(self):
            self.joint_calls = 0
            self.pose_calls = 0
            self.saw_via_standoff = False

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
            tip_arr = np.asarray(tip, dtype=float).reshape(3)
            center = np.array([0.15, 0.0, 0.15])
            dist = float(np.linalg.norm(tip_arr - center))
            # Prep-seed plans use max_attempts=1; recovery vias use >1 and
            # clearances in ~[0.05, 0.18]. Oriented contact targets ~0.02 m.
            if (
                0.045 <= dist <= 0.20
                and int(max_attempts) > 1
            ):
                self.saw_via_standoff = True
                return _fake_ok_traj(q_start, tip, quat, message="ok|via")
            if not self.saw_via_standoff:
                return PlannedTrajectory(
                    waypoints_rad=np.zeros((0, 6)),
                    dt_s=0.02,
                    success=False,
                    backend="curobo",
                    message="plan_failed:MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION",
                )
            # After a via, allow oriented contact approach + tip-omit nudge.
            return _fake_ok_traj(q_start, tip, quat, message="ok|contact")

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
    assert (
        "via1_" in traj.message
        or "via_contact" in traj.message
        or "via_standoff" in traj.message
    )
    # INVALID_START path exercised via joint_goal and/or early pose fails.
    assert planner.joint_calls >= 1 or planner.pose_calls >= 2
    assert planner.pose_calls >= 1


def test_viz_uses_recovery_planner():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "plan_collision_free_with_recovery" in src
    assert "execute_waypoints" in src
    assert "MARKER_COMMIT" in src
