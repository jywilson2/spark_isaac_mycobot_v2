# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Multi-seed IK reseeding for failed numerical solves.

Why
---
A single seed (often ``q_current``) can trap damped least-squares IK in a
local minimum, joint-limit wall, or poor null-space branch. Industrial stacks
(MoveIt ``kinematics_solver_attempts``, cached IK) and recent work such as
IKSel rank **joint-space** seeds and, after a failure, prefer candidates far
from the failed set — not a random Cartesian point farther from the target.

This module is classical IK support for Phase 2 sequential multi-target
sequences (``spec.md`` § Sequential multi-target / IK failure repositioning).
It does **not** replace collision-aware planning vias or bounded residuals.

Units: joint angles in radians.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose
from residual_adaptive_ik.kinematics.ik_base import IKResult, IKSolver
from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad


def _as_q6(q: np.ndarray | Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=float).reshape(-1)
    if arr.size != 6:
        raise ValueError(f"seed must have length 6, got {arr.size}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("seed contains non-finite values")
    return arr


def joint_distance_rad(a: np.ndarray, b: np.ndarray) -> float:
    """L2 distance between two joint vectors (radians)."""
    return float(np.linalg.norm(_as_q6(a) - _as_q6(b)))


def build_seed_bank(
    q_current: np.ndarray,
    *,
    home_q: np.ndarray | None = None,
    previous_goals: Sequence[np.ndarray] | None = None,
    extra_seeds: Sequence[np.ndarray] | None = None,
    include_mid_home: bool = True,
) -> list[np.ndarray]:
    """Assemble a deduplicated joint-space seed bank (radians).

    Order of insertion (later reordered by ``order_seeds``):

    1. ``q_current`` — continuity for sequential multi-target
    2. previous successful goals (most recent first)
    3. home / retract
    4. mid-blend toward home (gentle preparatory pose)
    5. caller ``extra_seeds`` (workspace samples, cached IK, …)
    """
    bank: list[np.ndarray] = []
    seen: list[np.ndarray] = []

    def _add(q: np.ndarray | Sequence[float]) -> None:
        cand = _as_q6(q)
        for prev in seen:
            if float(np.linalg.norm(cand - prev)) < 1e-4:
                return
        seen.append(cand.copy())
        bank.append(cand.copy())

    _add(q_current)
    if previous_goals:
        for g in reversed(list(previous_goals)):
            _add(g)
    home = (
        _as_q6(home_q)
        if home_q is not None
        else load_home_joint_positions_rad()
    )
    _add(home)
    if include_mid_home:
        q_cur = _as_q6(q_current)
        _add(0.5 * (q_cur + home))
    if extra_seeds:
        for s in extra_seeds:
            _add(s)
    return bank


def order_seeds(
    seeds: Sequence[np.ndarray],
    *,
    q_current: np.ndarray,
    failed_seeds: Sequence[np.ndarray] | None = None,
) -> list[np.ndarray]:
    """Order seeds for the next IK attempt.

    * No failures yet: nearest to ``q_current`` in joint space (continuity).
    * After failures: among remaining seeds, prefer those **farthest** from the
      nearest failed seed (IKSel-style escape), then nearer to ``q_current`` as
      a tie-break.

    Random Cartesian tip jitter is intentionally not used here.
    """
    q_cur = _as_q6(q_current)
    remaining = [_as_q6(s) for s in seeds]
    failed = [_as_q6(f) for f in (failed_seeds or [])]

    def _near_failed(s: np.ndarray) -> bool:
        return any(float(np.linalg.norm(s - f)) < 1e-4 for f in failed)

    remaining = [s for s in remaining if not _near_failed(s)]
    if not remaining:
        return []

    if not failed:
        return sorted(remaining, key=lambda s: joint_distance_rad(s, q_cur))

    def _key(s: np.ndarray) -> tuple[float, float]:
        dist_fail = min(float(np.linalg.norm(s - f)) for f in failed)
        # Sort by descending distance-to-failed, then ascending to current.
        return (-dist_fail, joint_distance_rad(s, q_cur))

    return sorted(remaining, key=_key)


def solve_with_seed_bank(
    solver: IKSolver,
    target_pose: Pose,
    seeds: Sequence[np.ndarray],
    *,
    q_current: np.ndarray | None = None,
    max_seeds: int | None = None,
) -> IKResult:
    """Try classical IK with ordered seeds until one converges.

    Returns the first successful ``IKResult``, or the last failure (with a
    combined reason) if all seeds fail. Never invents an unsafe success.
    """
    q_ref = _as_q6(q_current) if q_current is not None else _as_q6(seeds[0])
    ordered = order_seeds(seeds, q_current=q_ref, failed_seeds=None)
    if max_seeds is not None:
        ordered = ordered[: max(0, int(max_seeds))]

    failed: list[np.ndarray] = []
    last: IKResult | None = None
    reasons: list[str] = []

    while ordered:
        seed = ordered[0]
        result = solver.solve(target_pose, seed_q=seed)
        last = result
        if result.success:
            tag = f"seed_bank_ok|tried={len(failed) + 1}|{result.reason}"
            return IKResult(
                success=True,
                q=result.q.copy(),
                position_error_m=result.position_error_m,
                orientation_error_rad=result.orientation_error_rad,
                iterations=result.iterations,
                reason=tag,
            )
        reasons.append(str(result.reason))
        failed.append(seed.copy())
        ordered = order_seeds(seeds, q_current=q_ref, failed_seeds=failed)
        if max_seeds is not None:
            # Cap total attempts, not just the first ordering pass.
            if len(failed) >= int(max_seeds):
                break

    if last is None:
        n = 6
        return IKResult(
            success=False,
            q=np.full(n, np.nan),
            position_error_m=float("inf"),
            orientation_error_rad=float("inf"),
            iterations=0,
            reason="seed_bank_empty",
        )
    return IKResult(
        success=False,
        q=last.q.copy(),
        position_error_m=last.position_error_m,
        orientation_error_rad=last.orientation_error_rad,
        iterations=last.iterations,
        reason=(
            f"seed_bank_exhausted|n={len(failed)}|"
            + ",".join(reasons[-5:])
        ),
    )
