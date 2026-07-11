# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Isaac Lab vectorized residual-IK environment (Phase 3).

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

class ResidualIKEnv:
    """Custom Isaac Lab env: classical IK base + learned residual action.

    RL training must run on the Isaac Sim host (not inside the Isaac ROS
    container). See ``scripts/host/`` for host delegation.
    """

    def reset(self, *args, **kwargs):
        raise NotImplementedError("Phase 3: implement Isaac Lab env")

    def step(self, actions):
        raise NotImplementedError("Phase 3: implement Isaac Lab env")
