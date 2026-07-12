# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Hardware-speed joint motion helpers (pure NumPy; no Isaac required)."""
from __future__ import annotations

import math

import numpy as np

from residual_adaptive_ik.kinematics.workspace_sampling import load_workspace_config


def test_max_joint_speed_matches_vendor_160_deg_s():
    env = load_workspace_config()
    assert env.max_joint_speed_deg_s == 160.0
    assert abs(env.max_joint_speed_rad_s - math.radians(160.0)) < 1e-12


def test_interpolation_respects_speed_cap():
    """Δq / Δt must not exceed vendor max when stepping like the viz loop."""
    max_speed = load_workspace_config().max_joint_speed_rad_s
    dt = 1.0 / 60.0
    q = np.zeros(6)
    q_goal = np.array([1.0, -0.5, 0.8, 0.0, 0.3, -0.2])
    step_limit = max_speed * dt
    for _ in range(10_000):
        err = q_goal - q
        max_err = float(np.max(np.abs(err)))
        if max_err < 1e-4:
            break
        scale = min(1.0, step_limit / max_err)
        dq = err * scale
        assert float(np.max(np.abs(dq))) <= step_limit + 1e-12
        q = q + dq
    assert np.allclose(q, q_goal, atol=1e-3)
