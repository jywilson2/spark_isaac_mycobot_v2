# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 planning package — collision-checked joint paths + cuRobo.

See ``spec.md`` Phase 2, ``joint_path.py`` (CI NumPy), and ``curobo_planner.py``
(NVIDIA cuRobo Apache-2.0 on the DGX Spark host).
"""
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    PlannedTrajectory,
    check_ground_collision,
    curobo_available,
    plan_collision_free,
)
from residual_adaptive_ik.planning.joint_path import (
    JointPath,
    interpolate_joint_path,
    plan_joint_lerp_checked,
)

__all__ = [
    "CuRoboMotionPlanner",
    "JointPath",
    "PlannedTrajectory",
    "check_ground_collision",
    "curobo_available",
    "interpolate_joint_path",
    "plan_collision_free",
    "plan_joint_lerp_checked",
]
