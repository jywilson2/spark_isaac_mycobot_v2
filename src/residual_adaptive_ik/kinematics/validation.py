# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Deterministic validation of candidate joint solutions.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose


@dataclass(frozen=True)
class ValidationResult:
    """Outcome of safety / limit / FK-error checks before execution."""

    ok: bool
    reasons: tuple[str, ...]


def validate_solution(
    q: np.ndarray,
    target_pose: Pose,
    config: dict[str, Any],
    *,
    q_ik: np.ndarray | None = None,
    residual_q: np.ndarray | None = None,
) -> ValidationResult:
    """Validate joint limits, residual bounds, FK pose error, and optional safety.

    Never silently ignore failures — callers must fall back or reject.
    """
    raise NotImplementedError("Phase 1: implement validation — see spec.md")
