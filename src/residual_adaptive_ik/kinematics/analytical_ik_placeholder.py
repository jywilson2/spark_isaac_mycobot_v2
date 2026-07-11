# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Placeholder for analytical / IKFast solvers.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

from residual_adaptive_ik.kinematics.fk import Pose
from residual_adaptive_ik.kinematics.ik_base import IKResult, IKSolver


class AnalyticalIKPlaceholder(IKSolver):
    """Interface-compatible stub for future IKFast or closed-form integration."""

    def solve(self, target_pose: Pose, seed_q=None) -> IKResult:
        raise NotImplementedError(
            "Analytical IK not wired yet; use DampedLeastSquaresIK for Phase 1."
        )
