# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Generate supervised residual datasets with simulated imperfections.

Phase 3 (``spec.md``): classical IK is the base; labels are small joint
corrections ``Δq`` that recover from simulated calibration / noise errors.

Perturbation types (SI units: meters, radians):
  - joint_bias: per-joint encoder offset
  - tool_frame: EE tool-frame position offset
  - base_frame: base translation perturbation
  - target_noise: Gaussian noise on the commanded target
  - payload_sag: simple downward tip error

Output: ``.npz`` with stacked arrays matching ``ResidualIKSample``.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from residual_adaptive_ik.data.dataset_schema import ResidualIKSample
from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import (
    DampedLeastSquaresIK,
    load_ik_solver_config,
    load_joint_limits_rad,
)
from residual_adaptive_ik.utils.math_utils import DEG2RAD, clamp_residual
from residual_adaptive_ik.utils.transforms import orientation_error_magnitude_rad


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_supervised_config(path: Path | str | None = None) -> dict[str, Any]:
    """Load ``configs/learning/supervised_residual.yaml``."""
    cfg_path = (
        Path(path)
        if path is not None
        else _repo_root() / "configs" / "learning" / "supervised_residual.yaml"
    )
    with cfg_path.open(encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


def sample_to_arrays(samples: list[ResidualIKSample]) -> dict[str, np.ndarray]:
    """Stack ``ResidualIKSample`` list into NPZ-ready arrays."""
    if not samples:
        raise ValueError("empty sample list")
    return {
        "target_position": np.stack([s.target_position for s in samples]),
        "target_quaternion": np.stack([s.target_quaternion for s in samples]),
        "q_current": np.stack([s.q_current for s in samples]),
        "q_ik": np.stack([s.q_ik for s in samples]),
        "observed_position_error": np.stack(
            [s.observed_position_error for s in samples]
        ),
        "observed_orientation_error": np.stack(
            [s.observed_orientation_error for s in samples]
        ),
        "delta_q_label": np.stack([s.delta_q_label for s in samples]),
    }


def pack_observation(sample: ResidualIKSample) -> np.ndarray:
    """Build the MLP observation vector (float64, shape ``(obs_dim,)``).

    Layout (meters / radians):
      target_xyz(3) + target_quat_wxyz(4) + q_current(6) + q_ik(6)
      + pos_err(3) + ori_err_scalar(1)  → 23
    """
    ori = np.asarray(sample.observed_orientation_error, dtype=float).reshape(-1)
    ori_scalar = float(ori.reshape(-1)[0]) if ori.size >= 1 else 0.0
    return np.concatenate(
        [
            np.asarray(sample.target_position, dtype=float).reshape(3),
            np.asarray(sample.target_quaternion, dtype=float).reshape(4),
            np.asarray(sample.q_current, dtype=float).reshape(6),
            np.asarray(sample.q_ik, dtype=float).reshape(6),
            np.asarray(sample.observed_position_error, dtype=float).reshape(3),
            np.array([ori_scalar], dtype=float),
        ]
    )


OBS_DIM = 23


def _apply_perturbation(
    true_pose: Pose,
    q_true: np.ndarray,
    *,
    rng: np.random.Generator,
    mode: str,
    scale: float,
) -> tuple[Pose, np.ndarray, dict[str, Any]]:
    """Return (observed_target, q_biased_seed, metadata)."""
    meta: dict[str, Any] = {"perturbation": mode, "scale": float(scale)}
    pos = np.asarray(true_pose.position_m, dtype=float).reshape(3).copy()
    quat = np.asarray(true_pose.quaternion_wxyz, dtype=float).reshape(4).copy()
    q_seed = np.asarray(q_true, dtype=float).reshape(6).copy()

    if mode == "clean":
        return Pose(position_m=pos, quaternion_wxyz=quat), q_seed, meta

    if mode == "joint_bias":
        # Encoder bias: commanded joints ≠ true joints by a small offset.
        bias = rng.uniform(-scale, scale, size=6) * DEG2RAD
        q_seed = q_seed + bias
        meta["joint_bias_rad"] = bias
        return Pose(position_m=pos, quaternion_wxyz=quat), q_seed, meta

    if mode == "tool_frame":
        offset = rng.uniform(-scale, scale, size=3) * 0.001  # scale in mm → m
        pos = pos + offset
        meta["tool_offset_m"] = offset
        return Pose(position_m=pos, quaternion_wxyz=quat), q_seed, meta

    if mode == "base_frame":
        offset = rng.uniform(-scale, scale, size=3) * 0.001
        pos = pos + offset
        meta["base_offset_m"] = offset
        return Pose(position_m=pos, quaternion_wxyz=quat), q_seed, meta

    if mode == "target_noise":
        pos = pos + rng.normal(0.0, scale * 0.001, size=3)
        meta["target_noise_sigma_m"] = float(scale * 0.001)
        return Pose(position_m=pos, quaternion_wxyz=quat), q_seed, meta

    if mode == "payload_sag":
        sag = np.array([0.0, 0.0, -scale * 0.001], dtype=float)
        pos = pos + sag
        meta["payload_sag_m"] = sag
        return Pose(position_m=pos, quaternion_wxyz=quat), q_seed, meta

    raise ValueError(f"unknown perturbation mode: {mode}")


def generate_samples(
    n: int,
    *,
    seed: int = 0,
    modes: list[str] | None = None,
    residual_limit_deg: float = 0.5,
    stress: bool = False,
) -> list[ResidualIKSample]:
    """Generate ``n`` supervised residual samples.

    For each reachable pose:
      1. Solve classical IK for the *observed* (perturbed) target → ``q_ik``.
      2. Label ``Δq = clamp(q_true - q_ik)`` so ``q_ik + Δq ≈ q_true``.
      3. Record FK error of ``q_ik`` vs the true tip pose.
    """
    rng = np.random.default_rng(seed)
    if modes is None:
        modes = [
            "clean",
            "joint_bias",
            "tool_frame",
            "base_frame",
            "target_noise",
            "payload_sag",
        ]
    scale = 2.0 if stress else 1.0
    # Mode-specific base scales (deg for joint_bias, mm for others).
    mode_scale = {
        "clean": 0.0,
        "joint_bias": 0.5 * scale,  # degrees
        "tool_frame": 3.0 * scale,  # mm
        "base_frame": 2.0 * scale,
        "target_noise": 2.0 * scale,
        "payload_sag": 2.0 * scale,
    }
    ik_cfg = load_ik_solver_config()
    solver = DampedLeastSquaresIK.from_config(ik_cfg)
    lo, hi = load_joint_limits_rad()
    limit_rad = float(residual_limit_deg) * DEG2RAD

    samples: list[ResidualIKSample] = []
    attempts = 0
    max_attempts = max(n * 40, 200)
    while len(samples) < n and attempts < max_attempts:
        attempts += 1
        q_true = rng.uniform(lo, hi)
        true_tip = forward_kinematics(q_true)
        # Skip near-singular / near-base samples (horizontal reach too small).
        if float(np.hypot(true_tip.position_m[0], true_tip.position_m[1])) < 0.08:
            continue

        mode = modes[len(samples) % len(modes)]
        obs_target, q_seed, meta = _apply_perturbation(
            true_tip,
            q_true,
            rng=rng,
            mode=mode,
            scale=float(mode_scale.get(mode, 1.0)),
        )
        ik = solver.solve(obs_target, seed_q=q_seed)
        if not ik.success:
            continue
        q_ik = np.asarray(ik.q, dtype=float).reshape(6)
        delta = clamp_residual(q_true - q_ik, limit_rad)

        fk_ik = forward_kinematics(q_ik)
        pos_err = np.asarray(true_tip.position_m - fk_ik.position_m, dtype=float)
        ori_err = np.array(
            [
                orientation_error_magnitude_rad(
                    true_tip.quaternion_wxyz, fk_ik.quaternion_wxyz
                )
            ],
            dtype=float,
        )
        samples.append(
            ResidualIKSample(
                target_position=np.asarray(obs_target.position_m, dtype=float),
                target_quaternion=np.asarray(
                    obs_target.quaternion_wxyz, dtype=float
                ),
                q_current=q_seed,
                q_ik=q_ik,
                observed_position_error=pos_err,
                observed_orientation_error=ori_err,
                delta_q_label=delta,
                metadata=meta,
            )
        )
    if len(samples) < n:
        raise RuntimeError(
            f"only generated {len(samples)}/{n} samples after {attempts} attempts"
        )
    return samples


def write_npz(path: Path, samples: list[ResidualIKSample]) -> None:
    """Write stacked sample arrays to ``path`` (``.npz``)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = sample_to_arrays(samples)
    np.savez_compressed(path, **arrays)


def load_npz(path: Path | str) -> dict[str, np.ndarray]:
    """Load a supervised residual NPZ."""
    data = np.load(Path(path), allow_pickle=False)
    return {k: data[k] for k in data.files}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Generate Phase 3 supervised residual datasets")
    p.add_argument("--train", type=int, default=800, help="train sample count")
    p.add_argument("--val", type=int, default=200, help="validation sample count")
    p.add_argument("--test", type=int, default=200, help="test sample count")
    p.add_argument("--stress", type=int, default=200, help="stress sample count")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument(
        "--out-dir",
        type=Path,
        default=_repo_root() / "assets" / "datasets",
    )
    p.add_argument("--residual-limit-deg", type=float, default=0.5)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out = Path(args.out_dir)
    specs = [
        ("supervised_train.npz", args.train, False, args.seed),
        ("supervised_val.npz", args.val, False, args.seed + 1),
        ("supervised_test.npz", args.test, False, args.seed + 2),
        ("supervised_stress.npz", args.stress, True, args.seed + 3),
    ]
    for name, n, stress, seed in specs:
        print(f"Generating {name}: n={n} stress={stress} seed={seed}")
        samples = generate_samples(
            n,
            seed=seed,
            residual_limit_deg=float(args.residual_limit_deg),
            stress=stress,
        )
        write_npz(out / name, samples)
        # Quick stats
        d = np.stack([s.delta_q_label for s in samples])
        pe = np.array(
            [float(np.linalg.norm(s.observed_position_error)) for s in samples]
        )
        print(
            f"  wrote {len(samples)} samples → {out / name}\n"
            f"  |Δq|_rms={float(np.sqrt(np.mean(d**2))):.4e} rad  "
            f"pos_err_med={float(np.median(pe)):.4e} m"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
