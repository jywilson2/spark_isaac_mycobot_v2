# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 3 acceptance: FK torch parity + eval gate contracts."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from residual_adaptive_ik.data.generate_supervised_data import (
    PERTURBATION_MODE_IDS,
    generate_samples,
    sample_to_arrays,
)
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.learning.evaluate_supervised import (
    check_phase3_acceptance,
    evaluate_split,
    oracle_predictor,
    zero_predictor,
)

REPO = Path(__file__).resolve().parents[1]


def test_npz_includes_perturbation_mode():
    samples = generate_samples(12, seed=5, modes=["clean", "joint_bias"])
    data = sample_to_arrays(samples)
    assert "perturbation_mode" in data
    assert data["perturbation_mode"].dtype == np.uint8
    assert int(PERTURBATION_MODE_IDS["clean"]) in data["perturbation_mode"]


@pytest.mark.skipif(
    __import__("importlib").util.find_spec("torch") is None,
    reason="torch not installed",
)
def test_fk_torch_matches_numpy_fk():
    torch = pytest.importorskip("torch")
    from residual_adaptive_ik.learning.fk_torch import build_fk_module

    fk = build_fk_module()
    fk.eval()
    rng = np.random.default_rng(0)
    q = rng.uniform(-1.0, 1.0, size=(16, 6)).astype(np.float32)
    with torch.no_grad():
        tip_t = fk(torch.from_numpy(q)).numpy()
    for i in range(q.shape[0]):
        ref = forward_kinematics(q[i]).position_m
        assert np.linalg.norm(tip_t[i] - ref) < 1e-4


def test_check_phase3_acceptance_passes_on_strong_residual():
    samples = generate_samples(32, seed=9)
    data = sample_to_arrays(samples)
    ik = evaluate_split(data, zero_predictor, name="ik_only")
    mlp = evaluate_split(data, oracle_predictor, name="ik_plus_mlp_residual")
    summary = {
        "reports": [ik, mlp],
        "clean_subset": {},
    }
    acc = check_phase3_acceptance(summary)
    assert acc["ok_mlp_improvement"] is True


def test_train_supervised_documents_fk_loss():
    src = (REPO / "src" / "residual_adaptive_ik" / "learning" / "train_supervised.py").read_text(
        encoding="utf-8"
    )
    assert "lambda_fk" in src or "lambda_fk_pose" in src
    assert "build_fk_module" in src
    assert "joint_limit_penalty" in src


def test_evaluate_supervised_stress_and_acceptance_hooks():
    src = (
        REPO / "src" / "residual_adaptive_ik" / "learning" / "evaluate_supervised.py"
    ).read_text(encoding="utf-8")
    assert "stress_reports" in src
    assert "check_phase3_acceptance" in src
    assert "by_perturbation_mode" in src
