# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Train supervised residual IK model (Phase 3).

Composite loss (``spec.md``):
  MSE(Δq_pred, Δq_label)
  + λ_fk·||FK(q_ik+Δq) − true_tip||²
  + λ_mag·||Δq||²
  + λ_lim·joint_limit_penalty(q_ik+Δq)

Requires PyTorch. On the DGX Spark host::

    ./scripts/host/spark_host_exec.sh \\
      python3 -m residual_adaptive_ik.learning.train_supervised
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from residual_adaptive_ik.data.generate_supervised_data import (
    OBS_DIM,
    load_npz,
    load_supervised_config,
    pack_observation,
)
from residual_adaptive_ik.data.dataset_schema import ResidualIKSample
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.learning.fk_torch import (
    build_fk_module,
    true_tip_positions_numpy,
)
from residual_adaptive_ik.utils.math_utils import DEG2RAD


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _require_torch():
    try:
        import torch
        from torch import nn
        from torch.utils.data import DataLoader, TensorDataset
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "PyTorch is required for supervised residual training. "
            "Run on the Isaac Sim host python, or: pip install torch"
        ) from exc
    return torch, nn, DataLoader, TensorDataset


def arrays_to_obs_labels(
    data: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Convert NPZ arrays to (obs, delta_q_label, q_ik, true_tip_xyz)."""
    n = int(data["q_ik"].shape[0])
    obs = np.zeros((n, OBS_DIM), dtype=np.float32)
    labels = np.asarray(data["delta_q_label"], dtype=np.float32)
    q_ik = np.asarray(data["q_ik"], dtype=np.float32)
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
        obs[i] = pack_observation(sample).astype(np.float32)
    true_tip = true_tip_positions_numpy(q_ik, data["observed_position_error"])
    return obs, labels, q_ik, true_tip


def _joint_limit_penalty(
    q: "torch.Tensor", lo: "torch.Tensor", hi: "torch.Tensor"
) -> "torch.Tensor":
    """Soft penalty for joints outside ``[lo, hi]`` (radians)."""
    below = (lo - q).clamp(min=0.0)
    above = (q - hi).clamp(min=0.0)
    return (below**2 + above**2).sum(dim=-1).mean()


def train(
    *,
    cfg: dict[str, Any] | None = None,
    epochs: int | None = None,
    device: str | None = None,
) -> dict[str, Any]:
    """Train ``ResidualIKModel`` and write checkpoint + CSV metrics."""
    torch, nn, DataLoader, TensorDataset = _require_torch()
    from residual_adaptive_ik.learning.residual_model import ResidualIKModel

    cfg = cfg or load_supervised_config()
    model_cfg = cfg.get("model", {})
    train_cfg = cfg.get("training", {})
    loss_cfg = cfg.get("loss", {})
    ds_cfg = cfg.get("dataset", {})

    root = _repo_root()
    train_path = root / ds_cfg.get("train_path", "assets/datasets/supervised_train.npz")
    val_path = root / ds_cfg.get("val_path", "assets/datasets/supervised_val.npz")
    ckpt_dir = root / cfg.get("checkpoint_dir", "assets/checkpoints/supervised_residual")
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    if not train_path.is_file():
        raise FileNotFoundError(
            f"missing {train_path}; run: python -m residual_adaptive_ik.data.generate_supervised_data"
        )

    x_train, y_train, q_ik_train, tip_train = arrays_to_obs_labels(load_npz(train_path))
    x_val, y_val, q_ik_val, tip_val = arrays_to_obs_labels(load_npz(val_path))

    # Standardize observations (fit on train only).
    obs_mean = x_train.mean(axis=0)
    obs_std = x_train.std(axis=0)
    obs_std = np.where(obs_std < 1e-6, 1.0, obs_std).astype(np.float32)
    obs_mean = obs_mean.astype(np.float32)
    x_train_n = (x_train - obs_mean) / obs_std
    x_val_n = (x_val - obs_mean) / obs_std

    seed = int(train_cfg.get("seed", 42))
    torch.manual_seed(seed)
    np.random.seed(seed)

    dev = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    lo_np, hi_np = load_joint_limits_rad()
    lo_t = torch.tensor(lo_np, dtype=torch.float32, device=dev)
    hi_t = torch.tensor(hi_np, dtype=torch.float32, device=dev)
    fk = build_fk_module().to(dev)
    fk.eval()  # FK has no learnable weights; keep in eval for BatchNorm-free chain

    limit_rad = float(model_cfg.get("output_limit_deg", 0.5)) * DEG2RAD
    hidden = list(model_cfg.get("hidden_sizes", [256, 256, 128]))
    model = ResidualIKModel(OBS_DIM, hidden, limit_rad).to(dev)

    batch = int(train_cfg.get("batch_size", 256))
    n_epochs = int(epochs if epochs is not None else train_cfg.get("epochs", 100))
    lr = float(train_cfg.get("learning_rate", 1e-3))
    wd = float(train_cfg.get("weight_decay", 1e-5))
    lam_dq = float(loss_cfg.get("lambda_delta_q", 1.0))
    lam_fk = float(loss_cfg.get("lambda_fk_pose", 0.0))
    lam_mag = float(loss_cfg.get("lambda_residual_magnitude", 1e-4))
    lam_lim = float(loss_cfg.get("lambda_joint_limit", 0.0))

    q_ik_train_t = torch.from_numpy(q_ik_train).to(dev)
    tip_train_t = torch.from_numpy(tip_train).to(dev)
    q_ik_val_t = torch.from_numpy(q_ik_val).to(dev)
    tip_val_t = torch.from_numpy(tip_val).to(dev)

    train_loader = DataLoader(
        TensorDataset(
            torch.from_numpy(x_train_n),
            torch.from_numpy(y_train),
            q_ik_train_t,
            tip_train_t,
        ),
        batch_size=batch,
        shuffle=True,
    )
    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=wd)

    metrics_path = ckpt_dir / "train_metrics.csv"
    best_val = float("inf")
    best_path = ckpt_dir / "best.pt"
    history: list[dict[str, float]] = []

    with metrics_path.open("w", encoding="utf-8", newline="") as csv_f:
        writer = csv.DictWriter(
            csv_f,
            fieldnames=[
                "epoch",
                "train_loss",
                "val_loss",
                "val_mae_rad",
                "val_fk_m_median",
            ],
        )
        writer.writeheader()
        for epoch in range(1, n_epochs + 1):
            model.train()
            train_losses: list[float] = []
            for xb, yb, qik_b, tip_b in train_loader:
                xb = xb.to(dev)
                yb = yb.to(dev)
                qik_b = qik_b.to(dev)
                tip_b = tip_b.to(dev)
                pred = model(xb)
                loss = lam_dq * nn.functional.mse_loss(pred, yb)
                q_final = qik_b + pred
                if lam_fk > 0.0:
                    tip_pred = fk(q_final)
                    loss = loss + lam_fk * nn.functional.mse_loss(tip_pred, tip_b)
                if lam_lim > 0.0:
                    loss = loss + lam_lim * _joint_limit_penalty(q_final, lo_t, hi_t)
                loss = loss + lam_mag * (pred**2).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                train_losses.append(float(loss.item()))

            model.eval()
            with torch.no_grad():
                xv = torch.from_numpy(x_val_n).to(dev)
                yv = torch.from_numpy(y_val).to(dev)
                pv = model(xv)
                val_loss = lam_dq * nn.functional.mse_loss(pv, yv)
                qv = q_ik_val_t + pv
                if lam_fk > 0.0:
                    val_loss = val_loss + lam_fk * nn.functional.mse_loss(
                        fk(qv), tip_val_t
                    )
                if lam_lim > 0.0:
                    val_loss = val_loss + lam_lim * _joint_limit_penalty(
                        qv, lo_t, hi_t
                    )
                val_loss = val_loss + lam_mag * (pv**2).mean()
                val_loss_f = float(val_loss.item())
                val_mae = float(torch.mean(torch.abs(pv - yv)).item())
                tip_err = torch.linalg.norm(fk(qv) - tip_val_t, dim=-1)
                val_fk_med = float(torch.median(tip_err).item())

            row = {
                "epoch": float(epoch),
                "train_loss": float(np.mean(train_losses)),
                "val_loss": val_loss_f,
                "val_mae_rad": val_mae,
                "val_fk_m_median": val_fk_med,
            }
            history.append(row)
            writer.writerow(row)
            csv_f.flush()
            if val_loss_f < best_val:
                best_val = val_loss_f
                torch.save(
                    {
                        "model_state": model.state_dict(),
                        "obs_dim": OBS_DIM,
                        "hidden_sizes": hidden,
                        "output_limit_rad": limit_rad,
                        "obs_mean": obs_mean,
                        "obs_std": obs_std,
                        "epoch": epoch,
                        "val_loss": val_loss_f,
                    },
                    best_path,
                )
            if epoch == 1 or epoch % 10 == 0 or epoch == n_epochs:
                print(
                    f"epoch {epoch:3d}/{n_epochs}  train={row['train_loss']:.5f}  "
                    f"val={val_loss_f:.5f}  mae_rad={val_mae:.5e}  "
                    f"val_fk_med_m={val_fk_med:.5e}"
                )

    summary = {
        "best_val_loss": best_val,
        "epochs": n_epochs,
        "checkpoint": str(best_path),
        "metrics_csv": str(metrics_path),
        "n_train": int(x_train.shape[0]),
        "n_val": int(x_val.shape[0]),
        "device": str(dev),
    }
    (ckpt_dir / "train_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(f"Best checkpoint: {best_path} (val_loss={best_val:.5f})")
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Train Phase 3 supervised residual MLP")
    p.add_argument("--epochs", type=int, default=None)
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--device", type=str, default=None)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    cfg = load_supervised_config(args.config) if args.config else load_supervised_config()
    train(cfg=cfg, epochs=args.epochs, device=args.device)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
