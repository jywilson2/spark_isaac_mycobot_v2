# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Adapter around pymycobot / ROS joint command interfaces.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

class MyCobotDriverAdapter:
    """Hardware adapter — never used unless ENABLE_MYCOBOT_HARDWARE_TESTS=1."""

    def send_joint_positions(self, q_rad) -> None:
        raise NotImplementedError("ROS 2 hardware adapter — gated")
