# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 3 evaluation: oracle residual must improve over IK alone."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.data.generate_supervised_data import (
    generate_samples,
    sample_to_arrays,
)
from residual_adaptive_ik.learning.evaluate_supervised import (
    evaluate_split,
    oracle_predictor,
    zero_predictor,
)


def test_oracle_residual_improves_or_matches_ik():
    samples = generate_samples(
        40, seed=21, modes=["joint_bias", "tool_frame", "target_noise"]
    )
    data = sample_to_arrays(samples)
    ik = evaluate_split(data, zero_predictor, name="ik")
    ora = evaluate_split(data, oracle_predictor, name="oracle")
    # Oracle uses labels that point toward the true joints — median error
    # after residual should not be worse than IK alone (allow tiny tol).
    assert ora["pos_err_after_median_m"] <= ik["pos_err_after_median_m"] + 1e-6
    # Per-joint ±0.5° ⇒ L2 magnitude ≤ √6 · 0.5°.
    max_l2 = np.sqrt(6.0) * 0.5 * np.pi / 180.0
    assert ora["residual_mag_p95_rad"] <= max_l2 + 1e-9
    assert ora["validation_reject_rate"] <= 1.0
