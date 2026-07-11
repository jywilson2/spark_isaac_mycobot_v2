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
from pathlib import Path
from typing import Any

import numpy as np

from residual_adaptive_ik.kinematics.fk import forward_kinematics
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


def sample_reachable_poses(
    *,
    n: int,
    rng: np.random.Generator,
    model,
    joint_lower_rad: np.ndarray,
    joint_upper_rad: np.ndarray,
    workspace_radius_m: float | None,
) -> tuple[list, np.ndarray]:
    """Return ``(poses, q_true)`` for ``n`` samples inside joint limits.

    When ``workspace_radius_m`` is set, reject samples whose EE position lies
    outside the working ball (rejection sampling).
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
            if float(np.linalg.norm(pose.position_m)) > workspace_radius_m:
                continue
        poses.append(pose)
        qs.append(q)
    if len(poses) < n:
        raise RuntimeError(
            f"only sampled {len(poses)}/{n} reachable poses after {attempts} attempts"
        )
    return poses, np.asarray(qs, dtype=float)


def evaluate_baseline(
    *,
    n_poses: int = 1000,
    seed: int = 0,
    urdf_path: Path | str | None = None,
    seed_jitter_rad: float = 0.15,
    filter_workspace: bool = True,
) -> dict[str, Any]:
    """Run ≥``n_poses`` IK solves and return metrics dict (JSON-serializable)."""
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

    poses, q_true = sample_reachable_poses(
        n=n_poses,
        rng=rng,
        model=model,
        joint_lower_rad=lo,
        joint_upper_rad=hi,
        workspace_radius_m=radius,
    )

    pos_errors: list[float] = []
    ori_errors: list[float] = []
    iterations: list[int] = []
    times_s: list[float] = []
    success_flags: list[bool] = []
    failure_reasons: Counter[str] = Counter()
    validation_ok = 0

    for i, target in enumerate(poses):
        # Perturb seed away from truth so the solver must iterate.
        jitter = rng.uniform(-seed_jitter_rad, seed_jitter_rad, size=6)
        seed_q = np.clip(q_true[i] + jitter, lo, hi)
        t0 = time.perf_counter()
        result = solver.solve(target, seed_q=seed_q)
        dt = time.perf_counter() - t0
        times_s.append(dt)
        iterations.append(result.iterations)
        pos_errors.append(result.position_error_m)
        ori_errors.append(result.orientation_error_rad)
        success_flags.append(bool(result.success))
        if not result.success:
            failure_reasons[result.reason] += 1
        else:
            # Successful solves must also pass validation.
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
                # Spec: never report success for invalid q — treat as failure.
                success_flags[-1] = False

    pos = np.asarray(pos_errors, dtype=float)
    ori = np.asarray(ori_errors, dtype=float)
    succ = np.asarray(success_flags, dtype=bool)
    # Errors for successful solves only (standard IK reporting)
    pos_ok = pos[succ] if np.any(succ) else np.array([np.nan])
    ori_ok = ori[succ] if np.any(succ) else np.array([np.nan])

    metrics: dict[str, Any] = {
        "n_poses": n_poses,
        "seed": seed,
        "urdf": str(urdf_path),
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
            "real-world accuracy without hardware measurement."
        ),
    }
    return metrics


def metrics_to_markdown(metrics: dict[str, Any]) -> str:
    """Render a human-readable Phase 1 baseline report."""
    fails = metrics.get("failure_categories") or {}
    fail_lines = "\n".join(f"- `{k}`: {v}" for k, v in sorted(fails.items())) or "- (none)"
    return f"""# Phase 1 — Classical IK Baseline

Status: **complete** (URDF FK + DLS IK + validation; sim metrics only)

Generated by `residual_adaptive_ik.kinematics.baseline_eval` / `scripts/run_phase1_baseline.sh`.

## Metrics

| Metric | Value |
|--------|------:|
| Poses evaluated | {metrics['n_poses']} |
| Success rate | {metrics['success_rate']:.4f} ({metrics['n_success']}/{metrics['n_poses']}) |
| Median position error | {metrics['median_position_error_m']:.6e} m |
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
```

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
