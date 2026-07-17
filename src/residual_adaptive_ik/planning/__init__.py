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
from residual_adaptive_ik.planning.recovery import (
    plan_collision_free_with_recovery,
    plan_via_standoff,
    recovery_audit_plan_fail,
    tip_standoff_on_approach,
    try_move_to_preparatory_seed,
    try_oriented_tip_face_contact,
)
from residual_adaptive_ik.planning.contact_geometry import (
    ContactApproach,
    build_sphere_contact_approach,
    tool_axis_from_quaternion,
    validate_axial_contact_segment,
)

__all__ = [
    "ContactApproach",
    "CuRoboMotionPlanner",
    "JointPath",
    "PlannedTrajectory",
    "build_sphere_contact_approach",
    "check_ground_collision",
    "curobo_available",
    "interpolate_joint_path",
    "plan_collision_free",
    "plan_collision_free_with_recovery",
    "plan_joint_lerp_checked",
    "plan_via_standoff",
    "recovery_audit_plan_fail",
    "tip_standoff_on_approach",
    "tool_axis_from_quaternion",
    "try_move_to_preparatory_seed",
    "try_oriented_tip_face_contact",
    "validate_axial_contact_segment",
]
