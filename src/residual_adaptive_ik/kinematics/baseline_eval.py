# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 1 classical IK baseline evaluation over reachable FK poses.

Samples joint configurations within limits, uses FK as ground-truth targets,
then re-solves with DLS IK from a perturbed seed. Metrics are sim/FK-model
accuracy only — not real-world hardware claims. See ``spec.md`` Phase 1.
"""
from __future__ import annotations

import argparse
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import (
    DampedLeastSquaresIK,
    load_ik_solver_config,
    load_joint_limits_rad,
)
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.kinematics.validation import load_validation_config, validate_solution
from residual_adaptive_ik.utils.logging_utils import write_json


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class BaselineTrial:
    """One reachable-target IK trial (for metrics and optional Isaac viz)."""

    index: int
    target: Pose
    q_true: np.ndarray
    seed_q: np.ndarray
    q_sol: np.ndarray
    success: bool
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    solve_time_s: float
    reason: str


def sample_reachable_poses(
    *,
    n: int,
    rng: np.random.Generator,
    model,
    joint_lower_rad: np.ndarray,
    joint_upper_rad: np.ndarray,
    workspace_radius_m: float | None,
) -> tuple[list, np.ndarray]:
    """Return ``(poses, q_true)`` via joint-space sampling (legacy / fallback).

    When ``workspace_radius_m`` is set, reject samples whose EE position lies
    outside the working ball (rejection sampling). Prefer
    ``sample_stratified_target_poses`` for even Cartesian coverage.
    """
    poses = []
    qs = []
    attempts = 0
    max_attempts = n * 50
    while len(poses) < n and attempts < max_attempts:
        attempts += 1
        q = rng.uniform(joint_lower_rad, joint_upper_rad)
        pose = forward_kinematics(q, model=model)
        if workspace_radius_m is not None:
            # Horizontal reach (vendor working radius), not 3D ball norm
            if float(np.hypot(pose.position_m[0], pose.position_m[1])) > workspace_radius_m:
                continue
        poses.append(pose)
        qs.append(q)
    if len(poses) < n:
        raise RuntimeError(
            f"only sampled {len(poses)}/{n} reachable poses after {attempts} attempts"
        )
    return poses, np.asarray(qs, dtype=float)


def sample_even_workspace_reachable_poses(
    *,
    n: int,
    rng: np.random.Generator,
    model,
    joint_lower_rad: np.ndarray,
    joint_upper_rad: np.ndarray,
    envelope=None,
) -> tuple[list[Pose], np.ndarray]:
    """Reachable FK poses with even stratified bin coverage (v1-style).

    Samples joint configurations (so targets are reachable by construction) but
    only accepts poses whose EE lies in under-filled cylindrical workspace bins
    (azimuth × radius × z). This spreads IK targets across the arm's working
    envelope instead of clustering near singular joint draws.
    """
    from residual_adaptive_ik.kinematics.workspace_sampling import (
        assign_workspace_bin,
        load_workspace_config,
    )

    env = envelope or load_workspace_config()
    n_bins = env.n_bins
    quota = np.full(n_bins, n // n_bins, dtype=int)
    quota[: n % n_bins] += 1
    counts = np.zeros(n_bins, dtype=int)
    poses: list[Pose] = []
    qs: list[np.ndarray] = []
    attempts = 0
    max_attempts = max(n * 500, 10_000)
    while len(poses) < n and attempts < max_attempts:
        attempts += 1
        q = rng.uniform(joint_lower_rad, joint_upper_rad)
        pose = forward_kinematics(q, model=model)
        assigned = assign_workspace_bin(pose.position_m, env)
        if assigned is None:
            continue
        az_b, r_b, z_b = assigned
        idx = az_b * env.radius_bins * env.z_bins + r_b * env.z_bins + z_b
        if counts[idx] >= quota[idx]:
            continue
        counts[idx] += 1
        poses.append(pose)
        qs.append(np.asarray(q, dtype=float))
    if len(poses) < n:
        raise RuntimeError(
            f"even workspace sampling filled {len(poses)}/{n} poses after "
            f"{attempts} attempts (bin counts={counts.tolist()})"
        )
    return poses, np.asarray(qs, dtype=float)


def sample_stratified_target_poses(
    *,
    n: int,
    rng: np.random.Generator,
    model,
    joint_lower_rad: np.ndarray,
    joint_upper_rad: np.ndarray,
    envelope=None,
) -> tuple[list[Pose], np.ndarray]:
    """Alias kept for tests — even reachable coverage (see sample_even_*)."""
    return sample_even_workspace_reachable_poses(
        n=n,
        rng=rng,
        model=model,
        joint_lower_rad=joint_lower_rad,
        joint_upper_rad=joint_upper_rad,
        envelope=envelope,
    )


def evaluate_baseline(
    *,
    n_poses: int = 1000,
    seed: int = 0,
    urdf_path: Path | str | None = None,
    seed_jitter_rad: float = 0.15,
    filter_workspace: bool = True,
    return_trials: bool = False,
    sampling: str = "stratified",
) -> dict[str, Any] | tuple[dict[str, Any], list[BaselineTrial]]:
    """Run ≥``n_poses`` IK solves and return metrics (and optionally trials).

    ``sampling``:
      - ``\"stratified\"`` (default): reachable FK poses with even cylindrical
        bin coverage (v1-style) across the arm working envelope.
      - ``\"joints\"``: legacy unrestricted joint-space FK sampling (optional
        ball radius filter only).
    """
    root = _repo_root()
    if urdf_path is None:
        urdf_path = root / "assets" / "urdf" / "mycobot_280_m5_kinematics.urdf"
    model = load_urdf_model(urdf_path)
    lo, hi = load_joint_limits_rad()
    val_cfg = load_validation_config()
    ik_cfg = load_ik_solver_config()
    solver = DampedLeastSquaresIK.from_config(ik_cfg, model=model)
    rng = np.random.default_rng(seed)
    radius = float(val_cfg["workspace_radius_m"]) if filter_workspace else None

    if sampling == "stratified":
        poses, q_true = sample_even_workspace_reachable_poses(
            n=n_poses,
            rng=rng,
            model=model,
            joint_lower_rad=lo,
            joint_upper_rad=hi,
        )
    elif sampling == "joints":
        poses, q_true = sample_reachable_poses(
            n=n_poses,
            rng=rng,
            model=model,
            joint_lower_rad=lo,
            joint_upper_rad=hi,
            workspace_radius_m=radius,
        )
    else:
        raise ValueError(f"unknown sampling mode: {sampling!r}")

    pos_errors: list[float] = []
    ori_errors: list[float] = []
    iterations: list[int] = []
    times_s: list[float] = []
    success_flags: list[bool] = []
    failure_reasons: Counter[str] = Counter()
    validation_ok = 0
    trials: list[BaselineTrial] = []

    for i, target in enumerate(poses):
        jitter = rng.uniform(-seed_jitter_rad, seed_jitter_rad, size=6)
        seed_q = np.clip(q_true[i] + jitter, lo, hi)
        t0 = time.perf_counter()
        result = solver.solve(target, seed_q=seed_q)
        dt = time.perf_counter() - t0
        times_s.append(dt)
        iterations.append(result.iterations)
        pos_errors.append(result.position_error_m)
        ori_errors.append(result.orientation_error_rad)
        ok = bool(result.success)
        reason = result.reason
        if not result.success:
            failure_reasons[result.reason] += 1
        else:
            v = validate_solution(
                result.q,
                target,
                val_cfg,
                q_ik=result.q,
                residual_q=np.zeros(6),
                model=model,
                joint_lower_rad=lo,
                joint_upper_rad=hi,
            )
            if v.ok:
                validation_ok += 1
            else:
                failure_reasons["validation_failed_after_success"] += 1
                ok = False
                reason = "validation_failed_after_success"
        success_flags.append(ok)
        if return_trials:
            trials.append(
                BaselineTrial(
                    index=i,
                    target=target,
                    q_true=q_true[i].copy(),
                    seed_q=seed_q.copy(),
                    q_sol=result.q.copy(),
                    success=ok,
                    position_error_m=float(result.position_error_m),
                    orientation_error_rad=float(result.orientation_error_rad),
                    iterations=int(result.iterations),
                    solve_time_s=float(dt),
                    reason=reason,
                )
            )

    pos = np.asarray(pos_errors, dtype=float)
    ori = np.asarray(ori_errors, dtype=float)
    succ = np.asarray(success_flags, dtype=bool)
    pos_ok = pos[succ] if np.any(succ) else np.array([np.nan])
    ori_ok = ori[succ] if np.any(succ) else np.array([np.nan])

    metrics: dict[str, Any] = {
        "n_poses": n_poses,
        "seed": seed,
        "urdf": str(urdf_path),
        "sampling": sampling,
        "filter_workspace_radius_m": radius,
        "seed_jitter_rad": seed_jitter_rad,
        "success_rate": float(np.mean(succ)),
        "n_success": int(np.sum(succ)),
        "n_failure": int(np.sum(~succ)),
        "validation_ok_among_reported_success": validation_ok,
        "median_position_error_m": float(np.nanmedian(pos_ok)),
        "p95_position_error_m": float(np.nanpercentile(pos_ok, 95)),
        "median_orientation_error_rad": float(np.nanmedian(ori_ok)),
        "p95_orientation_error_rad": float(np.nanpercentile(ori_ok, 95)),
        "mean_iterations": float(np.mean(iterations)),
        "mean_solve_time_s": float(np.mean(times_s)),
        "failure_categories": dict(failure_reasons),
        "ik_config": ik_cfg,
        "validation_config": val_cfg,
        "units": {
            "position_error": "meters",
            "orientation_error": "radians",
            "solve_time": "seconds",
            "joint_angles": "radians",
        },
        "note": (
            "Simulation / URDF-FK metrics only. Do not claim sub-millimeter "
            "real-world accuracy without hardware measurement. "
            "Default sampling fills cylindrical workspace bins evenly (v1-style)."
        ),
    }
    if return_trials:
        return metrics, trials
    return metrics


def select_trials_for_visualization(
    trials: list[BaselineTrial],
    *,
    max_visualize: int,
) -> list[BaselineTrial]:
    """Pick a spatially spread subset (prefer successes; fill failures if needed)."""
    if max_visualize <= 0 or not trials:
        return []
    from residual_adaptive_ik.kinematics.workspace_sampling import (
        assign_workspace_bin,
        load_workspace_config,
    )

    env = load_workspace_config()
    successes = [t for t in trials if t.success]
    failures = [t for t in trials if not t.success]
    chosen: list[BaselineTrial] = []
    used_bins: set[tuple[int, int, int]] = set()

    def _try_add(pool: list[BaselineTrial]) -> None:
        for trial in pool:
            if len(chosen) >= max_visualize:
                return
            assigned = assign_workspace_bin(trial.target.position_m, env)
            if assigned is not None and assigned in used_bins and len(chosen) < env.n_bins:
                continue
            if assigned is not None:
                used_bins.add(assigned)
            chosen.append(trial)

    _try_add(successes)
    if len(chosen) < max_visualize:
        _try_add([t for t in successes if t not in chosen])
    if len(chosen) < max_visualize:
        _try_add(failures)
    chosen.sort(key=lambda t: t.index)
    return chosen[:max_visualize]


def metrics_to_markdown(metrics: dict[str, Any]) -> str:
    """Render a human-readable Phase 1 baseline report."""
    fails = metrics.get("failure_categories") or {}
    fail_lines = "\n".join(f"- `{k}`: {v}" for k, v in sorted(fails.items())) or "- (none)"
    viz_n = metrics.get("n_visualized")
    viz_line = (
        f"| Trials visualized in Isaac Sim | {viz_n} |\n"
        if viz_n is not None
        else ""
    )
    source = metrics.get(
        "generated_by",
        "`residual_adaptive_ik.kinematics.baseline_eval` / `scripts/run_phase1_baseline.sh`",
    )
    host_cmd = ""
    if metrics.get("isaac_viz"):
        host_cmd = (
            "\n# Host (metrics + Isaac Sim rendering):\n"
            "./scripts/host/run_phase1_isaac.sh\n"
        )
    return f"""# Phase 1 — Classical IK Baseline

Status: **complete** (URDF FK + DLS IK + validation; sim metrics only)

Generated by {source}.

## Metrics

| Metric | Value |
|--------|------:|
| Poses evaluated | {metrics['n_poses']} |
| Success rate | {metrics['success_rate']:.4f} ({metrics['n_success']}/{metrics['n_poses']}) |
{viz_line}| Median position error | {metrics['median_position_error_m']:.6e} m |
| 95th %ile position error | {metrics['p95_position_error_m']:.6e} m |
| Median orientation error | {metrics['median_orientation_error_rad']:.6e} rad |
| 95th %ile orientation error | {metrics['p95_orientation_error_rad']:.6e} rad |
| Mean solver iterations | {metrics['mean_iterations']:.2f} |
| Mean solve time | {metrics['mean_solve_time_s']:.6e} s |

Workspace filter radius: `{metrics['filter_workspace_radius_m']}` m  
Seed jitter: `{metrics['seed_jitter_rad']}` rad  
RNG seed: `{metrics['seed']}`  
URDF: `{metrics['urdf']}`

### Failure categories

{fail_lines}

## Honesty note

{metrics['note']}

## Repro commands

```bash
cd /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
source scripts/source_container_env.sh   # or: export PYTHONPATH=src
pytest tests -q
bash scripts/run_phase1_baseline.sh
{host_cmd}```

JSON metrics: `assets/logs/phase1_baseline_metrics.json`
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Phase 1 DLS IK baseline evaluation")
    parser.add_argument("--n-poses", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--json-out",
        type=Path,
        default=_repo_root() / "assets" / "logs" / "phase1_baseline_metrics.json",
    )
    parser.add_argument(
        "--md-out",
        type=Path,
        default=_repo_root() / "docs" / "phase1_baseline.md",
    )
    parser.add_argument("--no-workspace-filter", action="store_true")
    args = parser.parse_args(argv)

    metrics = evaluate_baseline(
        n_poses=args.n_poses,
        seed=args.seed,
        filter_workspace=not args.no_workspace_filter,
    )
    write_json(args.json_out, metrics)
    args.md_out.parent.mkdir(parents=True, exist_ok=True)
    args.md_out.write_text(metrics_to_markdown(metrics))
    print(f"Wrote {args.json_out}")
    print(f"Wrote {args.md_out}")
    print(f"success_rate={metrics['success_rate']:.4f} n={metrics['n_poses']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
