# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 3 supervised dataset generation tests (no PyTorch required)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from residual_adaptive_ik.data.generate_supervised_data import (
    OBS_DIM,
    generate_samples,
    load_npz,
    pack_observation,
    sample_to_arrays,
    write_npz,
)
from residual_adaptive_ik.utils.math_utils import DEG2RAD


def test_generate_samples_shapes_and_bounds():
    samples = generate_samples(24, seed=7, residual_limit_deg=0.5)
    assert len(samples) == 24
    limit = 0.5 * DEG2RAD
    for s in samples:
        assert s.q_ik.shape == (6,)
        assert s.delta_q_label.shape == (6,)
        assert np.all(np.abs(s.delta_q_label) <= limit + 1e-12)
        obs = pack_observation(s)
        assert obs.shape == (OBS_DIM,)
        assert np.all(np.isfinite(obs))


def test_write_and_load_npz(tmp_path: Path):
    samples = generate_samples(12, seed=3)
    path = tmp_path / "tiny.npz"
    write_npz(path, samples)
    data = load_npz(path)
    assert data["q_ik"].shape == (12, 6)
    assert data["delta_q_label"].shape == (12, 6)
    assert data["perturbation_mode"].shape == (12,)
    arrays = sample_to_arrays(samples)
    assert arrays["target_position"].shape == (12, 3)


def test_clean_mode_has_small_residuals():
    samples = generate_samples(
        16, seed=11, modes=["clean"], residual_limit_deg=0.5
    )
    mags = [float(np.linalg.norm(s.delta_q_label)) for s in samples]
    # Clean targets: IK should nearly recover q_true → tiny labels.
    assert float(np.median(mags)) < 0.05


def test_joint_bias_produces_nonzero_labels():
    samples = generate_samples(
        20, seed=13, modes=["joint_bias"], residual_limit_deg=0.5
    )
    mags = np.array([float(np.linalg.norm(s.delta_q_label)) for s in samples])
    assert float(np.median(mags)) > 1e-4
