# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Runtime safety monitor for residual IK deployment.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

class SafetyMonitor:
    """Hard checks before publishing joint commands to hardware."""

    def allow_command(self, q_candidate, q_current, config) -> bool:
        raise NotImplementedError("Implement hardware safety gates — see spec.md")
