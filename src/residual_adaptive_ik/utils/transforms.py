# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""SE(3) / quaternion helpers with explicit units.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

import numpy as np


def quaternion_wxyz_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion ``(w, x, y, z)``."""
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    return q / n
