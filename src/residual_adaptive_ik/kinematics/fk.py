# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Forward kinematics for MyCobot 280.

Computes end-effector pose from a 6-DOF joint vector using the vendor URDF
kinematic chain (``g_base`` → ``joint6_flange``). See ``spec.md`` Phase 1.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from residual_adaptive_ik.kinematics.urdf_model import (
    UrdfKinematicModel,
    get_default_model,
    load_urdf_model,
)


@dataclass(frozen=True)
class Pose:
    """End-effector pose in the robot base frame.

    Units: position in meters, orientation as unit quaternion ``(w, x, y, z)``.
    """

    position_m: np.ndarray
    quaternion_wxyz: np.ndarray


def forward_kinematics(
    q: np.ndarray,
    *,
    model: UrdfKinematicModel | None = None,
    urdf_path: Path | str | None = None,
) -> Pose:
    """Return end-effector pose for 6-DOF joint vector ``q`` (radians).

    Loads the MyCobot 280 M5 URDF chain by default (third_party, assets, or
    workspace sibling ``mycobot_ros2``). Pass ``model`` or ``urdf_path`` to
    override. Pinocchio is optional later; this path is NumPy-only for CI.
    """
    if model is None:
        model = load_urdf_model(urdf_path) if urdf_path is not None else get_default_model()
    position_m, quaternion_wxyz = model.forward(q)
    return Pose(position_m=position_m, quaternion_wxyz=quaternion_wxyz)
