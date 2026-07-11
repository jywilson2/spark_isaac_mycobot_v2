# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Reward terms for residual SAC (Phase 3).

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

def compute_residual_reward(
    *,
    position_error_m: float,
    orientation_error_rad: float,
    delta_q,
    delta_q_prev,
    joint_limit_penalty: float,
    invalid: bool,
    weights: dict,
    success: bool,
) -> float:
    """Weighted pose / residual / smoothness / validity reward (see spec.md)."""
    raise NotImplementedError("Phase 3: implement reward")
