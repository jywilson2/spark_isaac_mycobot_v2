# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 planning: collision-checked joint-space paths (radians).

Why joint-space sampling (not full MoveIt/cuRobo yet)
-----------------------------------------------------
Phase 1 viz already moves with a speed-capped joint lerp. Phase 2 adds
**collision samples along that lerp** so proximal links cannot sweep through
a goal sphere before the tip arrives. That is the minimal planning layer that
makes ``validate_solution(..., collision=...)`` real.

Full GPU planners ([cuRobo](https://curobo.org/), Apache-2.0) or ROS 2 MoveIt 2
can replace / enrich this path generator later without changing the residual
contract ``q_final = q_ik + clamp(Δq)``. Prefer cuRobo on DGX Spark when you
need dense obstacle fields; prefer MoveIt when the primary consumer is a ROS 2
hardware stack. This module stays NumPy-only for CI.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from residual_adaptive_ik.geometry.collision import (
    CollisionReport,
    SphereObstacle,
    check_config_collision,
)
from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel


@dataclass(frozen=True)
class JointPath:
    """Discrete joint-space path.

    ``waypoints_rad`` shape ``(N, 6)`` in radians, including start and goal.
    """

    waypoints_rad: np.ndarray
    collision: CollisionReport

    @property
    def ok(self) -> bool:
        return not self.collision.collides


def interpolate_joint_path(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    n_samples: int = 24,
) -> np.ndarray:
    """Linear joint-space interpolation including endpoints (radians).

    ``n_samples`` is the number of intervals; returned array has ``n_samples+1``
    rows. This matches Phase 1 servo lerp geometry (configuration-space line).
    """
    q0 = np.asarray(q_start_rad, dtype=float).reshape(-1)
    q1 = np.asarray(q_goal_rad, dtype=float).reshape(-1)
    if q0.shape != q1.shape:
        raise ValueError(f"q shape mismatch: {q0.shape} vs {q1.shape}")
    n = max(1, int(n_samples))
    alphas = np.linspace(0.0, 1.0, n + 1)
    return np.stack([(1.0 - a) * q0 + a * q1 for a in alphas], axis=0)


def plan_joint_lerp_checked(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    obstacles: list[SphereObstacle],
    *,
    n_samples: int = 24,
    model: UrdfKinematicModel | None = None,
    link_radius_m: float = 0.025,
    ignore_tip_segment: bool = True,
) -> JointPath:
    """Plan a joint lerp and report collisions along intermediate samples.

    The **goal** configuration ignores the last two capsules (coarse tip/wrist)
    so a tip-in-sphere goal is allowed; intermediate samples still flag proximal
    link collisions with the marker. For mid-path samples, ``ignore_tip_segment``
    is False so wrist sweeps through the sphere are still reported.
    """
    waypoints = interpolate_joint_path(
        q_start_rad, q_goal_rad, n_samples=n_samples
    )
    reasons: list[str] = []
    for i, q in enumerate(waypoints):
        # Allow tip/wrist-in-sphere only on the final sample.
        ignore_tip = ignore_tip_segment and (i == len(waypoints) - 1)
        report = check_config_collision(
            q,
            obstacles,
            model=model,
            link_radius_m=link_radius_m,
            ignore_tip_segment=ignore_tip,
        )
        if report.collides:
            for r in report.reasons:
                reasons.append(f"sample_{i}:{r}")
    return JointPath(
        waypoints_rad=waypoints,
        collision=CollisionReport(
            collides=len(reasons) > 0, reasons=tuple(reasons)
        ),
    )
