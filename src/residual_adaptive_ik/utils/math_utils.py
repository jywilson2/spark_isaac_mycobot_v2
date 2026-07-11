# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Math helpers (deg/rad, clamping).

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

import numpy as np

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi


def clamp_residual(delta_q_rad: np.ndarray, max_residual_rad: float) -> np.ndarray:
    """Clamp residual joint corrections to ±max_residual_rad."""
    return np.clip(delta_q_rad, -max_residual_rad, max_residual_rad)
