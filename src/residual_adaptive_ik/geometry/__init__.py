# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 geometry package — collision models for validation and planning.

See ``spec.md`` Phase 2 and ``residual_adaptive_ik.geometry.collision``.
"""
from residual_adaptive_ik.geometry.collision import (
    Capsule,
    CollisionReport,
    SphereObstacle,
    capsule_sphere_collide,
    check_config_collision,
    link_capsules_from_q,
    point_segment_distance_m,
    proximal_arm_contacts_target,
)

__all__ = [
    "Capsule",
    "CollisionReport",
    "SphereObstacle",
    "capsule_sphere_collide",
    "check_config_collision",
    "link_capsules_from_q",
    "point_segment_distance_m",
    "proximal_arm_contacts_target",
]
