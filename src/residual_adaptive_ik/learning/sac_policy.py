# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""SAC policy wrapper for residual joint corrections (Phase 3).

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

class SacResidualPolicy:
    """Soft Actor-Critic policy whose action is bounded Δq, not absolute joints."""

    def act(self, obs, deterministic: bool = False):
        raise NotImplementedError("Phase 3: implement or wrap SAC — see spec.md")
