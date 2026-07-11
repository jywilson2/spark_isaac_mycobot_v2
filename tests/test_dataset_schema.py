# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Dataset schema smoke test."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.data.dataset_schema import ResidualIKSample


def test_residual_ik_sample_fields():
    sample = ResidualIKSample(
        target_position=np.zeros(3),
        target_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        q_current=np.zeros(6),
        q_ik=np.zeros(6),
        observed_position_error=np.zeros(3),
        observed_orientation_error=np.zeros(3),
        delta_q_label=np.zeros(6),
        metadata={"split": "train"},
    )
    assert sample.q_ik.shape == (6,)
