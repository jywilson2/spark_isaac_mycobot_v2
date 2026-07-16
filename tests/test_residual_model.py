# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""ResidualIKModel output bounds (requires PyTorch)."""
from __future__ import annotations

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from residual_adaptive_ik.data.generate_supervised_data import OBS_DIM
from residual_adaptive_ik.learning.residual_model import ResidualIKModel
from residual_adaptive_ik.utils.math_utils import DEG2RAD


def test_residual_model_outputs_are_bounded():
    limit = 0.5 * DEG2RAD
    model = ResidualIKModel(OBS_DIM, [32, 32], limit)
    model.eval()
    x = torch.randn(8, OBS_DIM)
    with torch.no_grad():
        y = model(x).numpy()
    assert y.shape == (8, 6)
    assert np.all(np.abs(y) <= limit + 1e-6)
