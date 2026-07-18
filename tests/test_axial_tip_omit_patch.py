# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Axial tip-omit tip-face patch when exact pierce is IK-hard."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import DampedLeastSquaresIK
from residual_adaptive_ik.kinematics.urdf_model import get_default_model
from residual_adaptive_ik.planning.contact_geometry import build_sphere_contact_approach
from residual_adaptive_ik.planning.recovery import (
    contact_orientation_candidates,
    plan_axial_tip_omit_lerp,
)


def test_axial_tip_omit_patch_recovers_hard_pierce():
    """Ep6-like target: exact pierce fails; tip-face patch must succeed."""
    mdl = get_default_model()
    center = np.array([0.192, 0.008, 0.119], dtype=float)
    radius = 0.012
    tip_start = np.array([0.024, 0.146, 0.148], dtype=float)
    approach = build_sphere_contact_approach(
        tip_start, center, radius, standoff_m=0.008, nudge_m=0.008
    )
    ik_loose = DampedLeastSquaresIK(
        max_iterations=200,
        damping=5e-3,
        position_tol_m=0.005,
        orientation_tol_rad=0.35,
        enforce_joint_limits=True,
        model=mdl,
    )
    rng = np.random.default_rng(0)
    q_stand = None
    for _ in range(80):
        seed = rng.uniform(-1.5, 1.5, size=6)
        sol = ik_loose.solve(
            Pose(approach.standoff_position_m, approach.quaternion_wxyz),
            seed_q=seed,
        )
        if sol.success:
            q_stand = sol.q
            break
    assert q_stand is not None, "standoff seed unreachable in test setup"

    no_patch = plan_axial_tip_omit_lerp(
        q_stand,
        approach.pierce_position_m,
        approach.quaternion_wxyz,
        model=mdl,
    )
    assert not no_patch.ok

    with_patch = plan_axial_tip_omit_lerp(
        q_stand,
        approach.pierce_position_m,
        approach.quaternion_wxyz,
        model=mdl,
        sphere_center_m=center,
        sphere_radius_m=radius,
        patch_lateral_max_m=0.004,
    )
    assert with_patch.ok, with_patch.message
    assert "patch" in with_patch.message
    tip = np.asarray(
        forward_kinematics(with_patch.waypoints_rad[-1], model=mdl).position_m,
        dtype=float,
    ).reshape(3)
    dist_c = float(np.linalg.norm(tip - center))
    assert abs(dist_c - radius) < 0.004
    assert float(np.linalg.norm(tip - approach.pierce_position_m)) <= 0.004


def test_axial_tip_omit_patch_ep24_edge_needs_4mm():
    """GUI Ep24-like edge target: tip-face 4 mm disk unlocks pierce IK."""
    mdl = get_default_model()
    center = np.array([0.218, 0.023, 0.096], dtype=float)
    radius = 0.012
    tip_start = np.array([0.063, 0.115, 0.144], dtype=float)
    approach = build_sphere_contact_approach(
        tip_start, center, radius, standoff_m=0.008, nudge_m=0.008
    )
    cfg = {
        "contact_orientation_cone_enabled": True,
        "contact_orientation_cone_max_rad": 0.26,
        "contact_orientation_cone_tilts": 2,
        "contact_orientation_cone_azimuths": 4,
    }
    quats = contact_orientation_candidates(
        approach,
        fallback_quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        cfg=cfg,
    )
    ik_loose = DampedLeastSquaresIK(
        max_iterations=200,
        damping=5e-3,
        position_tol_m=0.006,
        orientation_tol_rad=0.35,
        enforce_joint_limits=True,
        model=mdl,
    )
    rng = np.random.default_rng(1)
    ok_4 = False
    for quat in quats:
        q_stand = None
        for _ in range(60):
            seed = rng.uniform(-1.5, 1.5, size=6)
            sol = ik_loose.solve(
                Pose(approach.standoff_position_m, quat), seed_q=seed
            )
            if sol.success or sol.position_error_m < 0.006:
                q_stand = sol.q
                break
        if q_stand is None:
            continue
        t4 = plan_axial_tip_omit_lerp(
            q_stand,
            approach.pierce_position_m,
            quat,
            model=mdl,
            sphere_center_m=center,
            sphere_radius_m=radius,
            patch_lateral_max_m=0.004,
        )
        if t4.ok:
            ok_4 = True
            break
    assert ok_4, "expected a tip-face 4 mm patch success for Ep24-like target"
