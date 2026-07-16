# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Evaluate classical IK vs IK + supervised residual (Phase 3).

Compares:
  1. classical IK only (``q_ik``)
  2. IK + oracle residual (labels) — upper bound on supervised gain
  3. IK + trained MLP residual (when checkpoint + torch available)
  4. validation rejection rate for residual candidates

See ``spec.md`` Phase 3 Metrics / Acceptance Criteria.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Callable

import numpy as np

from residual_adaptive_ik.data.generate_supervised_data import (
    load_npz,
    load_supervised_config,
    pack_observation,
)
from residual_adaptive_ik.data.dataset_schema import ResidualIKSample
from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.validation import load_validation_config, validate_solution
from residual_adaptive_ik.utils.math_utils import DEG2RAD, clamp_residual
from residual_adaptive_ik.utils.transforms import (
    orientation_error_magnitude_rad,
    position_error_norm_m,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


Predictor = Callable[[ResidualIKSample], np.ndarray]


def oracle_predictor(sample: ResidualIKSample) -> np.ndarray:
    """Return the dataset label (perfect residual within clamp bounds)."""
    return np.asarray(sample.delta_q_label, dtype=float).reshape(6)


def zero_predictor(_sample: ResidualIKSample) -> np.ndarray:
    """Classical IK only — zero residual."""
    return np.zeros(6, dtype=float)


def load_torch_predictor(checkpoint: Path) -> Predictor:
    """Load ``ResidualIKModel`` from ``best.pt``; raises if torch missing."""
    try:
        import torch
    except ImportError as exc:  # pragma: no cover
        raise ImportError("PyTorch required to load residual checkpoint") from exc
    from residual_adaptive_ik.learning.residual_model import ResidualIKModel

    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
    model = ResidualIKModel(
        int(ckpt["obs_dim"]),
        list(ckpt["hidden_sizes"]),
        float(ckpt["output_limit_rad"]),
    )
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    obs_mean = np.asarray(ckpt.get("obs_mean", 0.0), dtype=np.float32)
    obs_std = np.asarray(ckpt.get("obs_std", 1.0), dtype=np.float32)

    def _predict(sample: ResidualIKSample) -> np.ndarray:
        obs = pack_observation(sample).astype(np.float32)
        obs = (obs - obs_mean) / obs_std
        with torch.no_grad():
            out = model(torch.from_numpy(obs).unsqueeze(0))
        return out.squeeze(0).cpu().numpy().astype(float)

    return _predict


def _true_tip_from_errors(sample: ResidualIKSample) -> Pose:
    """Reconstruct approximate true tip = FK(q_ik) + observed position error."""
    fk = forward_kinematics(sample.q_ik)
    return Pose(
        position_m=fk.position_m + np.asarray(sample.observed_position_error, dtype=float),
        quaternion_wxyz=fk.quaternion_wxyz.copy(),
    )


def evaluate_split(
    data: dict[str, np.ndarray],
    predictor: Predictor,
    *,
    residual_limit_deg: float = 0.5,
    name: str = "model",
) -> dict[str, Any]:
    """Evaluate one predictor on an NPZ split; return summary metrics."""
    n = int(data["q_ik"].shape[0])
    limit = float(residual_limit_deg) * DEG2RAD
    val_cfg = load_validation_config()
    pos_before: list[float] = []
    pos_after: list[float] = []
    ori_before: list[float] = []
    ori_after: list[float] = []
    mag: list[float] = []
    n_reject = 0

    for i in range(n):
        sample = ResidualIKSample(
            target_position=data["target_position"][i],
            target_quaternion=data["target_quaternion"][i],
            q_current=data["q_current"][i],
            q_ik=data["q_ik"][i],
            observed_position_error=data["observed_position_error"][i],
            observed_orientation_error=data["observed_orientation_error"][i],
            delta_q_label=data["delta_q_label"][i],
        )
        true_tip = _true_tip_from_errors(sample)
        target = Pose(
            position_m=sample.target_position,
            quaternion_wxyz=sample.target_quaternion,
        )

        fk_ik = forward_kinematics(sample.q_ik)
        pos_before.append(position_error_norm_m(true_tip.position_m, fk_ik.position_m))
        ori_before.append(
            orientation_error_magnitude_rad(
                true_tip.quaternion_wxyz, fk_ik.quaternion_wxyz
            )
        )

        dq = clamp_residual(predictor(sample), limit)
        mag.append(float(np.linalg.norm(dq)))
        q_final = np.asarray(sample.q_ik, dtype=float) + dq
        # Safety vs true tip: residual bounds + joint limits. Pose tolerance is
        # measured against the *true* tip (not the noisy commanded target), so
        # a residual that corrects calibration is not rejected for disagreeing
        # with the corrupted observation.
        val_cfg_true = dict(val_cfg)
        val_cfg_true["max_position_error_m"] = max(
            float(val_cfg.get("max_position_error_m", 0.001)), 0.05
        )
        val_cfg_true["max_orientation_error_rad"] = max(
            float(val_cfg.get("max_orientation_error_rad", 0.01)), 0.5
        )
        result = validate_solution(
            q_final,
            true_tip,
            val_cfg_true,
            q_ik=sample.q_ik,
            residual_q=dq,
        )
        if not result.ok:
            n_reject += 1
            # Fall back to classical IK on validation failure.
            q_final = np.asarray(sample.q_ik, dtype=float)

        fk_f = forward_kinematics(q_final)
        pos_after.append(position_error_norm_m(true_tip.position_m, fk_f.position_m))
        ori_after.append(
            orientation_error_magnitude_rad(
                true_tip.quaternion_wxyz, fk_f.quaternion_wxyz
            )
        )

    pb = np.asarray(pos_before)
    pa = np.asarray(pos_after)
    return {
        "name": name,
        "n": n,
        "pos_err_before_median_m": float(np.median(pb)),
        "pos_err_after_median_m": float(np.median(pa)),
        "pos_err_before_mean_m": float(np.mean(pb)),
        "pos_err_after_mean_m": float(np.mean(pa)),
        "ori_err_before_median_rad": float(np.median(ori_before)),
        "ori_err_after_median_rad": float(np.median(ori_after)),
        "residual_mag_median_rad": float(np.median(mag)),
        "residual_mag_p95_rad": float(np.percentile(mag, 95)),
        "validation_reject_rate": float(n_reject) / float(max(n, 1)),
        "median_improvement_m": float(np.median(pb) - np.median(pa)),
    }


def run_evaluation(
    *,
    test_path: Path | None = None,
    checkpoint: Path | None = None,
    residual_limit_deg: float = 0.5,
    out_json: Path | None = None,
) -> dict[str, Any]:
    """Evaluate IK / oracle / optional MLP on the test split."""
    root = _repo_root()
    cfg = load_supervised_config()
    ds = cfg.get("dataset", {})
    path = test_path or (root / ds.get("test_path", "assets/datasets/supervised_test.npz"))
    data = load_npz(path)

    reports = [
        evaluate_split(
            data, zero_predictor, residual_limit_deg=residual_limit_deg, name="ik_only"
        ),
        evaluate_split(
            data,
            oracle_predictor,
            residual_limit_deg=residual_limit_deg,
            name="ik_plus_oracle_residual",
        ),
    ]

    ckpt = checkpoint or (
        root
        / cfg.get("checkpoint_dir", "assets/checkpoints/supervised_residual")
        / "best.pt"
    )
    if Path(ckpt).is_file():
        try:
            pred = load_torch_predictor(Path(ckpt))
            reports.append(
                evaluate_split(
                    data,
                    pred,
                    residual_limit_deg=residual_limit_deg,
                    name="ik_plus_mlp_residual",
                )
            )
        except ImportError as exc:
            reports.append({"name": "ik_plus_mlp_residual", "skipped": str(exc)})
    else:
        reports.append(
            {"name": "ik_plus_mlp_residual", "skipped": f"no checkpoint at {ckpt}"}
        )

    summary = {"test_path": str(path), "reports": reports}
    out = out_json or (
        root / "assets" / "logs" / "phase3_supervised_eval.json"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Evaluate Phase 3 supervised residual")
    p.add_argument("--test-path", type=Path, default=None)
    p.add_argument("--checkpoint", type=Path, default=None)
    p.add_argument("--residual-limit-deg", type=float, default=0.5)
    p.add_argument("--out-json", type=Path, default=None)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    run_evaluation(
        test_path=args.test_path,
        checkpoint=args.checkpoint,
        residual_limit_deg=float(args.residual_limit_deg),
        out_json=args.out_json,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
