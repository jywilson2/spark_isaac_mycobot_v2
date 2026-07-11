# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Residual clamping tests — Phase 2 acceptance."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.utils.math_utils import DEG2RAD, clamp_residual


def test_clamp_residual_respects_half_degree_default():
    limit = 0.5 * DEG2RAD
    oversized = np.full(6, 5.0 * DEG2RAD)
    clamped = clamp_residual(oversized, limit)
    assert np.all(np.abs(clamped) <= limit + 1e-12)
