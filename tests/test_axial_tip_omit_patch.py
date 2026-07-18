# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Axial tip-omit tip-face patch when exact pierce is IK-hard."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import DampedLeastSquaresIK
from residual_adaptive_ik.kinematics.urdf_model import get_default_model
from residual_adaptive_ik.planning.contact_geometry import build_sphere_contact_approach
from residual_adaptive_ik.planning.recovery import plan_axial_tip_omit_lerp


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
        patch_lateral_max_m=0.003,
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
