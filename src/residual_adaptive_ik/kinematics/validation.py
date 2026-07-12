# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Deterministic validation of candidate joint solutions.

Every learned residual must pass these checks before execution. On failure,
callers fall back to classical IK, numerical refinement, or no motion —
never unsafe ``Δq``. See ``spec.md`` and ``configs/ik/validation.yaml``.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel, get_default_model
from residual_adaptive_ik.utils.math_utils import DEG2RAD
from residual_adaptive_ik.utils.transforms import (
    orientation_error_magnitude_rad,
    position_error_norm_m,
)

# Optional Phase 2 geometry (imported lazily-friendly for typing).
from residual_adaptive_ik.geometry.collision import (  # noqa: E402
    SphereObstacle,
    check_config_collision,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_validation_config(path: Path | str | None = None) -> dict[str, Any]:
    """Load ``configs/ik/validation.yaml``."""
    cfg_path = Path(path) if path is not None else _repo_root() / "configs" / "ik" / "validation.yaml"
    with cfg_path.open() as f:
        return dict(yaml.safe_load(f))


@dataclass(frozen=True)
class ValidationResult:
    """Outcome of safety / limit / FK-error checks before execution."""

    ok: bool
    reasons: tuple[str, ...]


def validate_solution(
    q: np.ndarray,
    target_pose: Pose,
    config: dict[str, Any],
    *,
    q_ik: np.ndarray | None = None,
    residual_q: np.ndarray | None = None,
    model: UrdfKinematicModel | None = None,
    joint_lower_rad: np.ndarray | None = None,
    joint_upper_rad: np.ndarray | None = None,
    collision: bool | None = None,
    obstacles: list[SphereObstacle] | None = None,
    link_radius_m: float | None = None,
    ignore_tip_segment: bool = True,
) -> ValidationResult:
    """Validate joint limits, residual bounds, FK pose error, and optional safety.

    Never silently ignore failures — callers must fall back or reject.

    Parameters
    ----------
    q:
        Candidate joint vector (radians), shape ``(6,)``.
    target_pose:
        Desired end-effector pose (meters / quaternion wxyz).
    config:
        Keys from ``configs/ik/validation.yaml`` (thresholds and flags).
    q_ik:
        Classical IK solution; if given without ``residual_q``, residual is ``q - q_ik``.
    residual_q:
        Explicit residual ``Δq`` (radians) to bound-check.
    collision:
        Optional external collision flag; ``True`` rejects when provided.
        Prefer ``obstacles`` (Phase 2) so geometry is computed, not asserted.
    obstacles:
        Optional sphere obstacles (meters). When provided and
        ``enforce_collision`` is not false in ``config``, runs capsule checks.
    link_radius_m:
        Capsule radius override (meters); default 0.025.
    ignore_tip_segment:
        Allow the wrist→EE capsule to occupy a goal sphere (tip contact).
    """
    reasons: list[str] = []
    q_arr = np.asarray(q, dtype=float).reshape(-1)

    if q_arr.shape != (6,):
        reasons.append(f"bad_shape:{q_arr.shape}")
        return ValidationResult(ok=False, reasons=tuple(reasons))

    reject_nan = bool(config.get("reject_nan", True))
    reject_inf = bool(config.get("reject_inf", True))
    if reject_nan and np.any(np.isnan(q_arr)):
        reasons.append("nan_in_q")
    if reject_inf and np.any(np.isinf(q_arr)):
        reasons.append("inf_in_q")

    tgt_pos = np.asarray(target_pose.position_m, dtype=float).reshape(-1)
    tgt_quat = np.asarray(target_pose.quaternion_wxyz, dtype=float).reshape(-1)
    if tgt_pos.shape != (3,) or tgt_quat.shape != (4,):
        reasons.append("bad_target_pose_shape")
    else:
        if reject_nan and (np.any(np.isnan(tgt_pos)) or np.any(np.isnan(tgt_quat))):
            reasons.append("nan_in_target_pose")
        if reject_inf and (np.any(np.isinf(tgt_pos)) or np.any(np.isinf(tgt_quat))):
            reasons.append("inf_in_target_pose")

    # Residual bounds (Phase 3/4 path); Phase 1 may omit residual fields.
    delta: np.ndarray | None = None
    if residual_q is not None:
        delta = np.asarray(residual_q, dtype=float).reshape(-1)
    elif q_ik is not None:
        q_ik_arr = np.asarray(q_ik, dtype=float).reshape(-1)
        if q_ik_arr.shape != (6,):
            reasons.append(f"bad_q_ik_shape:{q_ik_arr.shape}")
        else:
            delta = q_arr - q_ik_arr
    if delta is not None:
        if delta.shape != (6,):
            reasons.append(f"bad_residual_shape:{delta.shape}")
        else:
            max_deg = float(config.get("max_residual_deg", 0.5))
            max_rad = max_deg * DEG2RAD
            if not np.all(np.isfinite(delta)):
                reasons.append("nonfinite_residual")
            elif np.any(np.abs(delta) > max_rad + 1e-12):
                reasons.append("residual_bound_exceeded")

    if bool(config.get("enforce_joint_limits", True)):
        if joint_lower_rad is None or joint_upper_rad is None:
            lo, hi = load_joint_limits_rad()
            joint_lower_rad = lo if joint_lower_rad is None else joint_lower_rad
            joint_upper_rad = hi if joint_upper_rad is None else joint_upper_rad
        lo = np.asarray(joint_lower_rad, dtype=float).reshape(6)
        hi = np.asarray(joint_upper_rad, dtype=float).reshape(6)
        if np.any(q_arr < lo - 1e-9) or np.any(q_arr > hi + 1e-9):
            reasons.append("joint_limit_violation")

    # Skip FK checks if q is already non-finite — FK would raise.
    if np.all(np.isfinite(q_arr)) and tgt_pos.shape == (3,) and tgt_quat.shape == (4,):
        if model is None:
            model = get_default_model()
        try:
            pose = forward_kinematics(q_arr, model=model)
        except ValueError as exc:
            reasons.append(f"fk_failed:{exc}")
        else:
            max_pos = float(config.get("max_position_error_m", 0.001))
            max_ori = float(config.get("max_orientation_error_rad", 0.01))
            pos_err = position_error_norm_m(pose.position_m, tgt_pos)
            ori_err = orientation_error_magnitude_rad(pose.quaternion_wxyz, tgt_quat)
            if pos_err > max_pos:
                reasons.append("position_error_exceeded")
            if ori_err > max_ori:
                reasons.append("orientation_error_exceeded")

            if bool(config.get("enforce_workspace_radius", True)):
                radius = float(config.get("workspace_radius_m", 0.280))
                # Vendor "working radius" is horizontal reach (cylindrical), not a 3D ball.
                # Matches configs/robot/workspace.yaml / v1 annulus sampling.
                tgt_r = float(np.hypot(float(tgt_pos[0]), float(tgt_pos[1])))
                if tgt_r > radius + 1e-9:
                    reasons.append("workspace_radius_exceeded")
                min_z = config.get("workspace_min_z_m")
                max_z = config.get("workspace_max_z_m")
                if min_z is not None and float(tgt_pos[2]) < float(min_z) - 1e-9:
                    reasons.append("workspace_z_below_min")
                if max_z is not None and float(tgt_pos[2]) > float(max_z) + 1e-9:
                    reasons.append("workspace_z_above_max")

    if collision is True:
        reasons.append("collision")

    # Phase 2: geometry-backed collision (capsule links vs sphere obstacles).
    if (
        obstacles
        and bool(config.get("enforce_collision", True))
        and np.all(np.isfinite(q_arr))
    ):
        radius = (
            float(link_radius_m)
            if link_radius_m is not None
            else float(config.get("link_radius_m", 0.025))
        )
        report = check_config_collision(
            q_arr,
            list(obstacles),
            model=model,
            link_radius_m=radius,
            ignore_tip_segment=ignore_tip_segment,
        )
        if report.collides:
            reasons.append("collision")
            reasons.extend(report.reasons)

    return ValidationResult(ok=len(reasons) == 0, reasons=tuple(reasons))
