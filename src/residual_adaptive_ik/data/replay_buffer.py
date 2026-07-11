# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Replay buffer helpers for SAC residual training (Phase 3).

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

class ReplayBuffer:
    """Store transitions for off-policy residual SAC."""

    def __init__(self, capacity: int) -> None:
        self.capacity = capacity

    def add(self, *args, **kwargs) -> None:
        raise NotImplementedError("Phase 3: implement replay buffer")

    def sample(self, batch_size: int):
        raise NotImplementedError("Phase 3: implement replay buffer")
