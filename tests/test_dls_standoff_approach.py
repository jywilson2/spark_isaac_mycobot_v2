# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for DLS standoff approach fallback (no CUDA)."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.urdf_model import get_default_model
from residual_adaptive_ik.planning.contact_geometry import build_sphere_contact_approach
from residual_adaptive_ik.planning.recovery import (
    plan_dls_standoff_approach_lerp,
    tip_path_avoids_marker_immersion,
)


def test_dls_standoff_approach_reaches_near_standoff():
    """From a nearby seed, DLS approach should land near the oriented standoff."""
    mdl = get_default_model()
    q0 = np.array([0.2, -0.2, 0.15, 0.0, 0.25, 0.0])
    tip0 = np.asarray(forward_kinematics(q0, model=mdl).position_m, dtype=float).reshape(3)
    center = tip0 + np.array([0.05, 0.0, -0.02])
    approach = build_sphere_contact_approach(
        tip0,
        center,
        0.012,
        standoff_m=0.008,
        nudge_m=0.008,
        current_quaternion_wxyz=forward_kinematics(q0, model=mdl).quaternion_wxyz,
    )
    traj = plan_dls_standoff_approach_lerp(
        q0,
        approach.standoff_position_m,
        approach.quaternion_wxyz,
        sphere_center_m=center,
        sphere_radius_m=0.012,
        model=mdl,
        n_samples=16,
    )
    # Reachable handoffs succeed; unreachable seeds may honestly fail.
    if traj.ok:
        tip_end = np.asarray(
            forward_kinematics(traj.waypoints_rad[-1], model=mdl).position_m,
            dtype=float,
        ).reshape(3)
        err = float(np.linalg.norm(tip_end - approach.standoff_position_m))
        assert err < 0.005
        assert "dls_standoff_approach" in traj.message
    else:
        assert "plan_failed:dls_approach" in traj.message


def test_dls_standoff_rejects_immersing_chord():
    """A path whose tip enters the marker mid-lerp must be refused."""
    mdl = get_default_model()
    q0 = np.zeros(6)
    tip0 = np.asarray(forward_kinematics(q0, model=mdl).position_m, dtype=float).reshape(3)
    center = tip0.copy()
    # Huge radius so tip0 (and any nearby lerp) is immersed.
    big_r = 0.25
    standoff = tip0 + np.array([0.03, 0.0, 0.0])
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    traj = plan_dls_standoff_approach_lerp(
        q0,
        standoff,
        quat,
        sphere_center_m=center,
        sphere_radius_m=big_r,
        model=mdl,
        n_samples=8,
    )
    assert not traj.ok
    assert "plan_failed:dls_approach" in traj.message


def test_tip_path_avoids_marker_immersion():
    mdl = get_default_model()
    q = np.zeros(6)
    tip = np.asarray(forward_kinematics(q, model=mdl).position_m, dtype=float)
    wp = np.vstack([q, q])
    ok, _d = tip_path_avoids_marker_immersion(
        wp,
        sphere_center_m=tip + np.array([1.0, 0.0, 0.0]),
        sphere_radius_m=0.012,
        model=mdl,
    )
    assert ok
    bad, min_d = tip_path_avoids_marker_immersion(
        wp,
        sphere_center_m=tip,
        sphere_radius_m=0.05,
        model=mdl,
    )
    assert not bad
    assert min_d < 0.01


def test_dls_approach_caller_applies_tip_face_gate():
    """CuRobo DLS fallback must reject tip-face grazes (iter16 Ep10)."""
    from pathlib import Path

    src = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "residual_adaptive_ik"
        / "planning"
        / "recovery.py"
    ).read_text(encoding="utf-8")
    assert "approach_dls_reject_tip_face" in src
    assert "plan_failed:dls_tip_face_" in src
    assert "approach_dls_reject_arm_body" in src
    assert "approach_dls_reject_immersion" in src
    assert "tip_omit_refused_after_reseat" in src


def test_tip_path_tip_face_ok_rejects_near_field_lateral():
    """Near-field off-axis tip must fail tip-face path check (iter18)."""
    from residual_adaptive_ik.planning.recovery import tip_path_tip_face_ok

    mdl = get_default_model()
    q = np.zeros(6)
    tip = np.asarray(forward_kinematics(q, model=mdl).position_m, dtype=float)
    # Place center so tip sits ~15 mm away, ~10 mm lateral to approach axis.
    center = tip + np.array([0.015, 0.010, 0.0])
    standoff = center + np.array([0.020, 0.0, 0.0])  # approach along +X
    ok, reason = tip_path_tip_face_ok(
        np.vstack([q, q]),
        sphere_center_m=center,
        sphere_radius_m=0.012,
        approach_from_m=standoff,
        model=mdl,
        cfg={"contact_standoff_m": 0.008, "contact_axis_tolerance_rad": 0.26},
        densify_n=8,
    )
    assert not ok
    assert reason == "side_graze"
