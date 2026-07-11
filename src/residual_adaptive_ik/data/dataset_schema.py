# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Supervised residual IK dataset schema.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class ResidualIKSample:
    """One supervised residual-IK training example."""

    target_position: np.ndarray
    target_quaternion: np.ndarray
    q_current: np.ndarray
    q_ik: np.ndarray
    observed_position_error: np.ndarray
    observed_orientation_error: np.ndarray
    delta_q_label: np.ndarray
    metadata: dict[str, Any] = field(default_factory=dict)
