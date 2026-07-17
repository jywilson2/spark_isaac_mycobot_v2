#!/usr/bin/env python3
# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Host smoke: Pinocchio dexterity gate (no pytest / ROS plugins)."""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO))

import numpy as np

from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.planning.dexterity import (
    contact_pose_is_dexterous,
    resolve_prescreen_backend,
)
from residual_adaptive_ik.planning.pinocchio_ik import (
    pinocchio_available,
    solve_pose_ik,
)


def main() -> int:
    assert pinocchio_available(), "pinocchio not importable under this python"
    assert resolve_prescreen_backend("auto") == "pinocchio"
    lo, hi = load_joint_limits_rad()
    q = 0.5 * (lo + hi)
    pose = forward_kinematics(q)
    res = solve_pose_ik(
        pose.position_m,
        pose.quaternion_wxyz,
        seed_q=q,
        position_tol_m=1e-3,
        orientation_tol_rad=1e-2,
    )
    assert res.success, (res.reason, res.position_error_m, res.orientation_error_rad)
    far = contact_pose_is_dexterous([0.6, 0.0, 0.1], 0.02, n_seeds=8, backend="pinocchio")
    assert not far.feasible and far.classification == "outside_dexterous_region"
    # In-band FK tip should not be falsely skipped as position-unreachable.
    rng = np.random.default_rng(7)
    tip = None
    for _ in range(400):
        qq = rng.uniform(lo, hi)
        t = forward_kinematics(qq).position_m
        if 0.16 < float(np.linalg.norm(t)) < 0.24:
            tip = t
            break
    assert tip is not None
    near = contact_pose_is_dexterous(tip, 0.02, n_seeds=12, backend="pinocchio")
    assert near.classification != "unreachable_position", near
    print(
        "Pinocchio prescreen smoke OK:",
        f"identity_ik=ok far={far.classification} near={near.classification}",
        f"near_feasible={near.feasible} backend={near.backend}",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
