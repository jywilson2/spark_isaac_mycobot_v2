# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Abstract classical IK solver interface.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose


@dataclass(frozen=True)
class IKResult:
    """Result of a classical IK solve (radians, meters)."""

    success: bool
    q: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    reason: str


class IKSolver(ABC):
    """Classical IK provides the base solution; learning only adds residuals."""

    @abstractmethod
    def solve(self, target_pose: Pose, seed_q: np.ndarray | None = None) -> IKResult:
        raise NotImplementedError
