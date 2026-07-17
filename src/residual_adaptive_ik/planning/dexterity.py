# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Deterministic dexterity prescreen for oriented tip-face contact targets.

Why (spec.md Phase 2 — recommended step #1)
-------------------------------------------
A long strict-1.0 GUI run showed ~30% of *uniformly sampled* targets fail the
oriented contact plan. Investigation (STATUS.md, 2026-07-17) found these are not
false successes or planner bugs — they are **orientation-feasibility limits at
the reach envelope**: the required pad-facing contact pose is simply not
solvable by IK there. Counting them as ``PLAN_FAIL`` measures *sampler optimism*
rather than planner quality.

This module provides a **deterministic** (CPU, no GPU/cuRobo) check of whether a
target's oriented pad-facing contact pose is reachable within joint limits.
Preferred backend is **Pinocchio** (analytic FK/Jacobian DLS —
``planning/pinocchio_ik.py``) when importable (host Isaac Sim Python). CI falls
back to the NumPy DLS + FK-pool oracle. Targets that no seed / orientation /
tool-axis sign can reach are reported ``SKIPPED_UNREACHABLE`` and excluded from
the PLAN_OK gate.

Honesty guardrails
------------------
* With the **Pinocchio** backend, orientation-infeasible targets may be skipped
  (``skip_orientation_infeasible`` defaults True when Pinocchio is selected):
  Pinocchio's Jacobians make multi-seed DLS a trustworthy enough completeness
  oracle for this gate.
* With the **NumPy** fallback, orientation skip stays **off** by default — plain
  DLS over-skips reachable poses. Position/reach-unreachable is always skipped.
* Feasibility is necessary-ish, not identical to cuRobo collision-free
  plannability: a target that passes here may still ``PLAN_FAIL`` in cuRobo.
* Both tool-axis signs are tested (planner-frame vs URDF/FK +Z mismatch).

Units: meters / radians.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from residual_adaptive_ik.kinematics.fk import Pose
from residual_adaptive_ik.kinematics.numerical_ik import (
    DampedLeastSquaresIK,
    load_joint_limits_rad,
)
from residual_adaptive_ik.planning.contact_geometry import (
    contact_orientation_cone,
)


def load_reach_envelope_m(path: Path | str | None = None) -> tuple[float, float]:
    """Return ``(min_reach_m, max_reach_m)`` from ``configs/robot/workspace.yaml``."""
    import yaml

    if path is None:
        path = (
            Path(__file__).resolve().parents[3]
            / "configs"
            / "robot"
            / "workspace.yaml"
        )
    with Path(path).open(encoding="utf-8") as handle:
        raw = dict(yaml.safe_load(handle) or {})
    return float(raw.get("min_reach_m", 0.12)), float(raw.get("max_reach_m", 0.28))


def target_radial_distance_m(
    target_position_m: np.ndarray,
    *,
    base_position_m: np.ndarray = (0.0, 0.0, 0.0),
) -> float:
    """Euclidean distance (meters) from the arm base origin to the target."""
    t = np.asarray(target_position_m, dtype=float).reshape(3)
    b = np.asarray(base_position_m, dtype=float).reshape(3)
    return float(np.linalg.norm(t - b))


def is_in_dexterous_region(
    target_position_m: np.ndarray,
    *,
    min_reach_m: float,
    max_reach_m: float,
    margin_m: float = 0.02,
    base_position_m: np.ndarray = (0.0, 0.0, 0.0),
) -> tuple[bool, str]:
    """Is the target position inside the interior reach shell (dexterous region)?

    The "Dexterous Region" is the interior of the reach envelope shrunk by
    ``margin_m`` on both ends: ``[min_reach + margin, max_reach - margin]``.
    Positions inside it are comfortably reachable, so a contact failure there is
    attributable to **orientation** rather than raw reach. Returns
    ``(in_region, reason)``.
    """
    radial = target_radial_distance_m(
        target_position_m, base_position_m=base_position_m
    )
    lo = float(min_reach_m) + float(margin_m)
    hi = float(max_reach_m) - float(margin_m)
    if radial < lo:
        return False, f"inside_min_reach:{radial:.3f}<{lo:.3f}"
    if radial > hi:
        return False, f"beyond_max_reach:{radial:.3f}>{hi:.3f}"
    return True, f"in_region:{lo:.3f}<={radial:.3f}<={hi:.3f}"


@dataclass
class DexterityResult:
    """Outcome of the oriented-contact reachability prescreen (SI units)."""

    feasible: bool
    classification: str  # "ok" | "unreachable_orientation" | "unreachable_position"
    radial_m: float
    in_region: bool
    region_reason: str
    best_pos_err_m: float
    best_ori_err_rad: float
    n_orientations_tried: int
    axis_sign: str = "none"  # "outward" | "inward" | "none"
    note: str = ""
    backend: str = "numpy"  # "pinocchio" | "numpy"

    def as_log_tokens(self) -> str:
        """Compact ``key=value`` tokens for the RESULT SKIPPED_UNREACHABLE line."""
        return (
            f"reason={self.classification} in_region={int(self.in_region)} "
            f"radial_m={self.radial_m:.3f} pos_err_m={self.best_pos_err_m:.4f} "
            f"ori_err_rad={self.best_ori_err_rad:.3f} "
            f"orientations={self.n_orientations_tried} axis={self.axis_sign} "
            f"backend={self.backend}"
        )


def resolve_prescreen_backend(requested: str = "auto") -> str:
    """Return ``pinocchio`` or ``numpy`` for the dexterity prescreen."""
    req = str(requested or "auto").strip().lower()
    if req == "numpy":
        return "numpy"
    if req == "pinocchio":
        from residual_adaptive_ik.planning.pinocchio_ik import pinocchio_available

        if not pinocchio_available():
            raise RuntimeError(
                "plan_prescreen_backend=pinocchio but pinocchio is not importable"
            )
        return "pinocchio"
    # auto
    from residual_adaptive_ik.planning.pinocchio_ik import pinocchio_available

    return "pinocchio" if pinocchio_available() else "numpy"


def _fk_sample_pool(
    model,
    lo: np.ndarray,
    hi: np.ndarray,
    *,
    n_samples: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Deterministic pool of ``(joint_configs, fk_tip_positions)``.

    Used both as a Monte-Carlo reachability oracle (is any FK tip near the
    pierce?) and as **warm-start seeds** for DLS refinement. Sampling +
    FK-nearest seeding makes DLS reliable for reachable poses, so an
    "infeasible" verdict is trustworthy (no over-skipping of real targets).
    """
    from residual_adaptive_ik.kinematics.fk import forward_kinematics

    rng = np.random.default_rng(20260717)  # fixed → deterministic prescreen
    qs = rng.uniform(lo, hi, size=(int(n_samples), lo.shape[0]))
    poses = [forward_kinematics(q, model=model) for q in qs]
    tips = np.array([p.position_m for p in poses], dtype=float)
    quats = np.array([p.quaternion_wxyz for p in poses], dtype=float)
    return qs, tips, quats


def _pose_nearest_seeds(
    qs: np.ndarray,
    tips: np.ndarray,
    quats: np.ndarray,
    target_m: np.ndarray,
    target_quat_wxyz: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    k: int,
    ori_weight_m_per_rad: float = 0.05,
) -> list[np.ndarray]:
    """Top-``k`` seeds minimizing a combined position + orientation cost.

    Orientation-aware warm starts make DLS converge reliably for *feasible*
    pad-facing poses, so an "infeasible" verdict is trustworthy (the prescreen
    must not over-skip genuinely reachable targets — that would dishonestly
    inflate the PLAN_OK gate).
    """
    from residual_adaptive_ik.utils.transforms import (
        orientation_error_magnitude_rad,
    )

    tgt = np.asarray(target_m, dtype=float).reshape(1, 3)
    pos_cost = np.linalg.norm(tips - tgt, axis=1)
    ori_cost = np.array(
        [
            orientation_error_magnitude_rad(q, target_quat_wxyz)
            for q in quats
        ],
        dtype=float,
    )
    cost = pos_cost + float(ori_weight_m_per_rad) * ori_cost
    order = np.argsort(cost)[: max(1, int(k))]
    seeds = [qs[i].copy() for i in order]
    seeds.append(0.5 * (lo + hi))
    return seeds


def build_prescreen_solver(
    *,
    position_tol_m: float = 0.006,
    orientation_tol_rad: float = 0.35,
) -> DampedLeastSquaresIK:
    """Stabilized DLS for the prescreen (large damping + step scaling).

    The Phase-1 metrics solver (damping 1e-3, step 1.0) is tuned for near-seed
    refinement and **diverges** on large-orientation-error targets, which would
    make it falsely report reachable poses as infeasible. This variant uses
    heavier damping and a small step so it descends stably from cold seeds.
    """
    return DampedLeastSquaresIK(
        max_iterations=400,
        damping=0.05,
        position_tol_m=float(position_tol_m),
        orientation_tol_rad=float(orientation_tol_rad),
        step_scale=0.3,
    )


def contact_pose_is_dexterous(
    target_position_m: np.ndarray,
    sphere_radius_m: float,
    *,
    solver: DampedLeastSquaresIK | None = None,
    base_position_m: np.ndarray = (0.0, 0.0, 0.0),
    standoff_m: float = 0.0,
    cone_max_rad: float = 0.30,
    n_tilts: int = 2,
    n_azimuths: int = 4,
    position_tol_m: float = 0.006,
    orientation_tol_rad: float = 0.35,
    n_seeds: int = 12,
    min_reach_m: float | None = None,
    max_reach_m: float | None = None,
    region_margin_m: float = 0.02,
    skip_orientation_infeasible: bool | None = None,
    backend: str = "auto",
) -> DexterityResult:
    """Deterministically test whether the pad-facing contact pose is reachable.

    Builds the near-side pierce point on the ``base→target`` ray and tries a
    bounded cone of pad-facing orientations (``contact_orientation_cone``) in
    **both** tool-axis signs, warm-started from FK-sampled seeds.

    Preferred backend is Pinocchio (``backend="auto"|"pinocchio"``). NumPy DLS
    is the CI fallback.

    Skip policy (honesty)
    ---------------------
    * ``unreachable_position`` ⇒ ``feasible=False`` always.
    * ``unreachable_orientation`` ⇒ skipped when ``skip_orientation_infeasible``
      is True. Default: **True for Pinocchio**, **False for NumPy** (NumPy DLS
      over-skips; Pinocchio Jacobians are trustworthy enough for this gate).
    """
    target = np.asarray(target_position_m, dtype=float).reshape(3)
    base = np.asarray(base_position_m, dtype=float).reshape(3)
    radius = float(sphere_radius_m)
    backend_resolved = resolve_prescreen_backend(backend)
    # Operator policy (spec.md): SKIPPED_UNREACHABLE is only for targets
    # outside the Dexterous Region. In-region orientation limits use vias /
    # recovery → PLAN_FAIL, never skip-as-unreachable. Default False.
    if skip_orientation_infeasible is None:
        skip_orientation_infeasible = False
    if solver is None:
        solver = build_prescreen_solver(
            position_tol_m=position_tol_m,
            orientation_tol_rad=orientation_tol_rad,
        )
    lo = np.asarray(solver.joint_lower_rad, dtype=float).reshape(-1)
    hi = np.asarray(solver.joint_upper_rad, dtype=float).reshape(-1)

    if min_reach_m is None or max_reach_m is None:
        try:
            mr, xr = load_reach_envelope_m()
        except Exception:
            mr, xr = 0.12, 0.28
        min_reach_m = mr if min_reach_m is None else min_reach_m
        max_reach_m = xr if max_reach_m is None else max_reach_m

    radial = target_radial_distance_m(target, base_position_m=base)
    in_region, region_reason = is_in_dexterous_region(
        target,
        min_reach_m=float(min_reach_m),
        max_reach_m=float(max_reach_m),
        margin_m=float(region_margin_m),
        base_position_m=base,
    )

    # Geometric dexterous-workspace gate: targets outside the interior reach
    # shell are sampler optimism (near-envelope orientation / path limits) and
    # are excluded from the PLAN_OK rate before any IK/planning work.
    if not in_region:
        return DexterityResult(
            feasible=False,
            classification="outside_dexterous_region",
            radial_m=radial,
            in_region=False,
            region_reason=region_reason,
            best_pos_err_m=float("nan"),
            best_ori_err_rad=float("nan"),
            n_orientations_tried=0,
            axis_sign="none",
            note=f"outside_dexterous_region|{region_reason}",
            backend=backend_resolved,
        )

    # Near-side approach direction on the base→target ray. The outward normal at
    # the pierce points from the sphere center back toward the base (reach side).
    delta = target - base
    dist = float(np.linalg.norm(delta))
    dir_bt = delta / dist if dist > 1e-9 else np.array([0.0, 0.0, 1.0], dtype=float)
    normal_outward = -dir_bt  # center → base (approach side)
    pierce = target + (radius + float(standoff_m)) * normal_outward

    # Both signs: the planner-frame +Z (outward) vs URDF/FK +Z (inward) mismatch
    # means the physically-correct contact may read either sign through FK. Test
    # both so feasibility does not depend on the bookkeeping convention.
    cone_out = contact_orientation_cone(
        normal_outward,
        cone_max_rad=cone_max_rad,
        n_tilts=n_tilts,
        n_azimuths=n_azimuths,
    )
    cone_in = contact_orientation_cone(
        -normal_outward,
        cone_max_rad=cone_max_rad,
        n_tilts=n_tilts,
        n_azimuths=n_azimuths,
    )
    oriented_candidates = [("outward", q) for q in cone_out] + [
        ("inward", q) for q in cone_in
    ]

    # FK-sampled pool: reachability oracle + DLS warm-start seeds. Pool size and
    # per-target seed count scale with ``n_seeds`` so the search is thorough
    # enough that an "infeasible" verdict is trustworthy (avoids over-skipping).
    pool_n = max(128, int(n_seeds) * 24)
    qs, tips, quats = _fk_sample_pool(solver.model, lo, hi, n_samples=pool_n)
    best_pos = float("inf")
    best_ori = float("inf")
    n_tried = 0
    k_seeds = max(4, int(n_seeds) // 2)

    # 1) Oriented feasibility: any (seed, orientation, sign) fully converges?
    # Seeds are orientation-aware per candidate (warm-started from the closest
    # full-pose pool sample). Pinocchio (preferred) or NumPy DLS.
    use_pinocchio = backend_resolved == "pinocchio"
    if use_pinocchio:
        from residual_adaptive_ik.planning.pinocchio_ik import multi_seed_pose_reachable

    for sign, quat in oriented_candidates:
        n_tried += 1
        seeds = _pose_nearest_seeds(
            qs, tips, quats, pierce, quat, lo=lo, hi=hi, k=k_seeds
        )
        if use_pinocchio:
            res_p = multi_seed_pose_reachable(
                pierce,
                quat,
                seeds=seeds,
                position_tol_m=position_tol_m,
                orientation_tol_rad=orientation_tol_rad,
            )
            best_pos = min(best_pos, float(res_p.position_error_m))
            best_ori = min(best_ori, float(res_p.orientation_error_rad))
            if res_p.success:
                return DexterityResult(
                    feasible=True,
                    classification="ok",
                    radial_m=radial,
                    in_region=in_region,
                    region_reason=region_reason,
                    best_pos_err_m=float(res_p.position_error_m),
                    best_ori_err_rad=float(res_p.orientation_error_rad),
                    n_orientations_tried=n_tried,
                    axis_sign=sign,
                    note="oriented_contact_reachable",
                    backend="pinocchio",
                )
            continue
        for seed in seeds:
            res = solver.solve(Pose(position_m=pierce, quaternion_wxyz=quat), seed_q=seed)
            best_pos = min(best_pos, float(res.position_error_m))
            best_ori = min(best_ori, float(res.orientation_error_rad))
            if res.success:
                return DexterityResult(
                    feasible=True,
                    classification="ok",
                    radial_m=radial,
                    in_region=in_region,
                    region_reason=region_reason,
                    best_pos_err_m=float(res.position_error_m),
                    best_ori_err_rad=float(res.orientation_error_rad),
                    n_orientations_tried=n_tried,
                    axis_sign=sign,
                    note="oriented_contact_reachable",
                    backend="numpy",
                )

    # 2) Position-only reachability. First a Monte-Carlo oracle over the FK pool
    # (is any sampled tip near the pierce?), then a DLS refine seeded from the
    # nearest sample with a permissive orientation. Reliable because the seed is
    # already close, so a "position unreachable" verdict is trustworthy.
    pool_min_dist = float(np.min(np.linalg.norm(tips - pierce.reshape(1, 3), axis=1)))
    pos_reachable = pool_min_dist <= max(0.03, float(position_tol_m) * 3.0)
    if pos_reachable:
        permissive = DampedLeastSquaresIK(
            max_iterations=solver.max_iterations,
            damping=solver.damping,
            position_tol_m=float(position_tol_m),
            orientation_tol_rad=float(np.pi),  # ignore orientation
            enforce_joint_limits=solver.enforce_joint_limits,
            joint_lower_rad=lo,
            joint_upper_rad=hi,
            model=solver.model,
        )
        refined = False
        pos_seeds = _pose_nearest_seeds(
            qs, tips, quats, pierce, oriented_candidates[0][1],
            lo=lo, hi=hi, k=k_seeds,
        )
        for seed in pos_seeds:
            res = permissive.solve(
                Pose(position_m=pierce, quaternion_wxyz=oriented_candidates[0][1]),
                seed_q=seed,
            )
            if float(res.position_error_m) <= float(position_tol_m):
                refined = True
                break
        # Trust the pool oracle even if DLS refine stalls — the sample proves a
        # config exists within pool resolution.
        pos_reachable = refined or pool_min_dist <= max(0.03, float(position_tol_m) * 3.0)

    classification = "unreachable_orientation" if pos_reachable else "unreachable_position"
    # Honesty gate: always skip provably out-of-reach targets; skip
    # orientation-limited ones only when explicitly opted in (DLS is not a
    # trustworthy orientation-completeness oracle — do not over-skip by default).
    if classification == "unreachable_position":
        feasible = False
    else:
        feasible = not bool(skip_orientation_infeasible)
    return DexterityResult(
        feasible=feasible,
        classification=classification,
        radial_m=radial,
        in_region=in_region,
        region_reason=region_reason,
        best_pos_err_m=best_pos,
        best_ori_err_rad=best_ori,
        n_orientations_tried=n_tried,
        axis_sign="none",
        note=("position_reachable_orientation_infeasible" if pos_reachable
              else "position_unreachable"),
        backend=backend_resolved,
    )
