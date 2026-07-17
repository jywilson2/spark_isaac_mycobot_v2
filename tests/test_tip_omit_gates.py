# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Tip-omit length + pad-alignment gates (no CUDA).

See ``spec.md`` Tip-face contact planning and ``planning/recovery.py``:
long tip-omit with flange spheres off lets the EE barrel hit the marker
from the side/back — refuse unless length ≤ ``contact_nudge_max_m`` and FK
tool +Z is already pad-aligned.
"""
from __future__ import annotations

import numpy as np
import pytest

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.urdf_model import get_default_model
from residual_adaptive_ik.planning.contact_geometry import (
    build_sphere_contact_approach,
    tool_axis_alignment_error_rad,
)
from residual_adaptive_ik.planning.curobo_planner import PlannedTrajectory
from residual_adaptive_ik.planning.recovery import (
    fk_pad_aligned_for_tip_omit,
    tip_omit_length_allow_m,
    try_oriented_tip_face_contact,
)


def test_tip_omit_length_allow_m_never_expands_past_nudge_max():
    """≤60 mm / large via defaults must not widen the tip-omit window."""
    assert tip_omit_length_allow_m(
        {"contact_via_nudge_max_m": 0.10}, nudge_max_m=0.012
    ) == pytest.approx(0.012)
    assert tip_omit_length_allow_m(
        {"contact_via_nudge_max_m": 0.008}, nudge_max_m=0.012
    ) == pytest.approx(0.008)
    assert tip_omit_length_allow_m({}, nudge_max_m=0.012) == pytest.approx(
        0.012
    )


def test_fk_pad_aligned_rejects_side_facing():
    mdl = get_default_model()
    q = np.zeros(6)
    pose = forward_kinematics(q, model=mdl)
    # Desired axis orthogonal to current tool +Z → large error.
    tool_z = pose.quaternion_wxyz  # use rotation of identity-ish home
    from residual_adaptive_ik.planning.contact_geometry import (
        tool_axis_from_quaternion,
    )

    z = tool_axis_from_quaternion(pose.quaternion_wxyz)
    # Pick a direction ~90° from tool Z.
    side = np.cross(z, np.array([0.0, 0.0, 1.0]))
    if float(np.linalg.norm(side)) < 1e-6:
        side = np.cross(z, np.array([0.0, 1.0, 0.0]))
    side = side / float(np.linalg.norm(side))
    cfg = {
        "contact_axis_tolerance_rad": 0.35,
        "contact_orientation_cone_enabled": True,
        "contact_orientation_cone_max_rad": 0.30,
    }
    ok, err, tol = fk_pad_aligned_for_tip_omit(
        q, side, model=mdl, cfg=cfg
    )
    assert not ok
    assert err > tol


def test_tip_omit_fallback_refuses_long_gap(monkeypatch):
    """After spheres-ON approach fail, tip 5 cm out must refuse tip-omit."""
    monkeypatch.setenv(
        "RESIDUAL_ADAPTIVE_IK_PLANNING_CONFIG",
        "",  # use default collision.yaml if set; force via load patch below
    )
    from residual_adaptive_ik.planning import recovery as recovery_mod

    cfg = {
        "contact_axis_enabled": True,
        "contact_standoff_m": 0.008,
        "contact_nudge_m": 0.008,
        "contact_nudge_max_m": 0.012,
        "contact_via_nudge_max_m": 0.012,
        "contact_lateral_tolerance_m": 0.003,
        "contact_axis_tolerance_rad": 0.35,
        "contact_orientation_cone_enabled": False,
        "target_obstacle_inflate_m": 0.0,
    }
    monkeypatch.setattr(recovery_mod, "load_planning_config", lambda: cfg)

    class _FailPlanner:
        _interpolation_dt_s = 0.02
        _omit_tip_links = False

        def plan_to_pose(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((1, 6)),
                dt_s=0.02,
                success=False,
                message="plan_failed:IK_FAIL",
                backend="fake",
            )

    class _OmitPlanner(_FailPlanner):
        _omit_tip_links = True

        def plan_to_pose(self, *args, **kwargs):
            # Must not be reached for a long gap.
            raise AssertionError("tip-omit must not run when tip_far > allow")

    q0 = np.zeros(6)
    tip = forward_kinematics(q0).position_m
    # Place sphere so pierce is ~5 cm from tip (outside nudge_max).
    center = tip + np.array([0.05, 0.0, 0.0])
    radius = 0.012
    log: list[str] = []
    out = try_oriented_tip_face_contact(
        q0,
        sphere_center_m=center,
        sphere_radius_m=radius,
        planner=_FailPlanner(),
        contact_planner=_OmitPlanner(),
        obstacles=[SphereObstacle(center_m=center, radius_m=radius, name="ik_target")],
        attempts_log=log,
    )
    assert out is None
    joined = "\n".join(log)
    assert "tip_omit_refused" in joined or "contact_nudge_direct_refused" in joined
    assert "0.060" not in joined  # old long allow must be gone
    # allow should be the short nudge cap
    assert any("allow_m=0.012" in line for line in log)


def test_tip_omit_fallback_refuses_misaligned_pad(monkeypatch):
    """Even a short tip→pierce gap refuses tip-omit when pad faces sideways."""
    from residual_adaptive_ik.planning import recovery as recovery_mod

    cfg = {
        "contact_axis_enabled": True,
        "contact_standoff_m": 0.008,
        "contact_nudge_m": 0.008,
        "contact_nudge_max_m": 0.012,
        "contact_via_nudge_max_m": 0.012,
        "contact_lateral_tolerance_m": 0.003,
        "contact_axis_tolerance_rad": 0.20,
        "contact_orientation_cone_enabled": True,
        "contact_orientation_cone_max_rad": 0.20,
        "target_obstacle_inflate_m": 0.0,
    }
    monkeypatch.setattr(recovery_mod, "load_planning_config", lambda: cfg)

    class _FailPlanner:
        _interpolation_dt_s = 0.02
        _omit_tip_links = False

        def plan_to_pose(self, *args, **kwargs):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((1, 6)),
                dt_s=0.02,
                success=False,
                message="plan_failed:IK_FAIL",
                backend="fake",
            )

    omit_calls = {"n": 0}

    class _OmitPlanner:
        _interpolation_dt_s = 0.02
        _omit_tip_links = True

        def plan_to_pose(self, *args, **kwargs):
            omit_calls["n"] += 1
            return PlannedTrajectory(
                waypoints_rad=np.zeros((2, 6)),
                dt_s=0.02,
                success=True,
                message="ok",
                backend="fake",
            )

    q0 = np.zeros(6)
    pose = forward_kinematics(q0)
    tip = np.asarray(pose.position_m, dtype=float).reshape(3)
    # Sphere almost under the tip so tip→pierce ≤ nudge_max, but place center
    # so the outward normal is orthogonal to current tool +Z when possible.
    from residual_adaptive_ik.planning.contact_geometry import (
        tool_axis_from_quaternion,
    )

    z = tool_axis_from_quaternion(pose.quaternion_wxyz)
    side = np.cross(z, np.array([1.0, 0.0, 0.0]))
    if float(np.linalg.norm(side)) < 1e-6:
        side = np.cross(z, np.array([0.0, 1.0, 0.0]))
    side = side / float(np.linalg.norm(side))
    # Center so tip is on the near side along ``side``, ~8 mm outside surface.
    radius = 0.012
    pierce = tip - 0.008 * side
    center = pierce - radius * side
    approach = build_sphere_contact_approach(tip, center, radius, standoff_m=0.008)
    assert float(np.linalg.norm(tip - approach.pierce_position_m)) <= 0.012 + 1e-3
    # Confirm pad is NOT aligned for this geometry.
    err = tool_axis_alignment_error_rad(pose.quaternion_wxyz, approach.normal_outward)
    assert err > 0.20

    log: list[str] = []
    out = try_oriented_tip_face_contact(
        q0,
        sphere_center_m=center,
        sphere_radius_m=radius,
        planner=_FailPlanner(),
        contact_planner=_OmitPlanner(),
        obstacles=[SphereObstacle(center_m=center, radius_m=radius, name="ik_target")],
        attempts_log=log,
    )
    assert out is None
    assert omit_calls["n"] == 0
    assert any("tip_omit_refused_orientation" in line for line in log)
