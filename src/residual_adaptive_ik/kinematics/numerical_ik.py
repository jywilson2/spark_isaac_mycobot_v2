# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Damped least-squares numerical IK baseline.

Implements ``q ← q + Jᵀ (J Jᵀ + λ² I)⁻¹ e`` with joint-limit clamping.
Classical IK is the deployed base; learning only adds residuals later
(``q_final = q_ik + clamp(Δq)``). See ``spec.md`` Phase 1.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.ik_base import IKResult, IKSolver
from residual_adaptive_ik.kinematics.urdf_model import (
    UrdfKinematicModel,
    get_default_model,
    load_urdf_model,
)
from residual_adaptive_ik.utils.math_utils import DEG2RAD
from residual_adaptive_ik.utils.transforms import (
    orientation_error_magnitude_rad,
    pose_error_6,
    position_error_norm_m,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_joint_limits_rad(
    path: Path | str | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Load lower/upper joint limits (radians) from ``configs/robot/joint_limits.yaml``."""
    cfg_path = Path(path) if path is not None else _repo_root() / "configs" / "robot" / "joint_limits.yaml"
    with cfg_path.open() as f:
        raw = yaml.safe_load(f)
    limits = raw["joint_limits_deg"]
    # Preserve joint1…joint6 order from mycobot_280.yaml
    keys = [f"joint{i}" for i in range(1, 7)]
    lo = np.array([limits[k][0] for k in keys], dtype=float) * DEG2RAD
    hi = np.array([limits[k][1] for k in keys], dtype=float) * DEG2RAD
    return lo, hi


def load_ik_solver_config(path: Path | str | None = None) -> dict[str, Any]:
    """Load ``configs/ik/ik_solver.yaml``."""
    cfg_path = Path(path) if path is not None else _repo_root() / "configs" / "ik" / "ik_solver.yaml"
    with cfg_path.open() as f:
        return dict(yaml.safe_load(f))


class DampedLeastSquaresIK(IKSolver):
    """Numerical IK with seed, damping, tolerances, and joint-limit enforcement."""

    def __init__(
        self,
        *,
        max_iterations: int = 100,
        damping: float = 1e-3,
        position_tol_m: float = 1e-3,
        orientation_tol_rad: float = 1e-2,
        enforce_joint_limits: bool = True,
        joint_lower_rad: np.ndarray | None = None,
        joint_upper_rad: np.ndarray | None = None,
        model: UrdfKinematicModel | None = None,
        urdf_path: Path | str | None = None,
        step_scale: float = 1.0,
    ) -> None:
        self.max_iterations = int(max_iterations)
        self.damping = float(damping)
        self.position_tol_m = float(position_tol_m)
        self.orientation_tol_rad = float(orientation_tol_rad)
        self.enforce_joint_limits = bool(enforce_joint_limits)
        self.step_scale = float(step_scale)
        if model is None:
            model = load_urdf_model(urdf_path) if urdf_path is not None else get_default_model()
        self.model = model
        if joint_lower_rad is None or joint_upper_rad is None:
            lo, hi = load_joint_limits_rad()
            self.joint_lower_rad = lo if joint_lower_rad is None else np.asarray(joint_lower_rad, dtype=float)
            self.joint_upper_rad = hi if joint_upper_rad is None else np.asarray(joint_upper_rad, dtype=float)
        else:
            self.joint_lower_rad = np.asarray(joint_lower_rad, dtype=float)
            self.joint_upper_rad = np.asarray(joint_upper_rad, dtype=float)

    @classmethod
    def from_config(
        cls,
        config: dict[str, Any] | None = None,
        *,
        urdf_path: Path | str | None = None,
        model: UrdfKinematicModel | None = None,
    ) -> DampedLeastSquaresIK:
        """Construct from ``ik_solver.yaml`` keys (or defaults)."""
        cfg = config if config is not None else load_ik_solver_config()
        return cls(
            max_iterations=int(cfg.get("max_iterations", 100)),
            damping=float(cfg.get("damping", 1e-3)),
            position_tol_m=float(cfg.get("position_tol_m", 1e-3)),
            orientation_tol_rad=float(cfg.get("orientation_tol_rad", 1e-2)),
            enforce_joint_limits=bool(cfg.get("enforce_joint_limits", True)),
            model=model,
            urdf_path=urdf_path,
        )

    def _clamp_q(self, q: np.ndarray) -> np.ndarray:
        if not self.enforce_joint_limits:
            return q
        return np.clip(q, self.joint_lower_rad, self.joint_upper_rad)

    def _errors(self, q: np.ndarray, target_pose: Pose) -> tuple[float, float, np.ndarray]:
        pose = forward_kinematics(q, model=self.model)
        pos_err = position_error_norm_m(pose.position_m, target_pose.position_m)
        ori_err = orientation_error_magnitude_rad(pose.quaternion_wxyz, target_pose.quaternion_wxyz)
        e6 = pose_error_6(
            pose.position_m,
            pose.quaternion_wxyz,
            target_pose.position_m,
            target_pose.quaternion_wxyz,
        )
        return pos_err, ori_err, e6

    def solve(self, target_pose: Pose, seed_q: np.ndarray | None = None) -> IKResult:
        """Solve for joint angles (radians) reaching ``target_pose``.

        Returns ``success=False`` with a reason when tolerances are not met; never
        reports success for a non-finite or limit-violating ``q``.
        """
        n = len(self.model.revolute_names)
        if seed_q is None:
            q = 0.5 * (self.joint_lower_rad + self.joint_upper_rad)
        else:
            q = np.asarray(seed_q, dtype=float).reshape(-1)
            if q.shape != (n,):
                return IKResult(
                    success=False,
                    q=np.full(n, np.nan),
                    position_error_m=float("inf"),
                    orientation_error_rad=float("inf"),
                    iterations=0,
                    reason=f"seed_q must have shape ({n},), got {q.shape}",
                )
            if not np.all(np.isfinite(q)):
                return IKResult(
                    success=False,
                    q=np.full(n, np.nan),
                    position_error_m=float("inf"),
                    orientation_error_rad=float("inf"),
                    iterations=0,
                    reason="seed_q contains non-finite values",
                )
        q = self._clamp_q(q.copy())

        tgt_pos = np.asarray(target_pose.position_m, dtype=float).reshape(3)
        tgt_quat = np.asarray(target_pose.quaternion_wxyz, dtype=float).reshape(4)
        if not (np.all(np.isfinite(tgt_pos)) and np.all(np.isfinite(tgt_quat))):
            return IKResult(
                success=False,
                q=q,
                position_error_m=float("inf"),
                orientation_error_rad=float("inf"),
                iterations=0,
                reason="target_pose contains non-finite values",
            )

        lam2 = self.damping * self.damping
        pos_err = float("inf")
        ori_err = float("inf")
        last_iter = 0

        for it in range(1, self.max_iterations + 1):
            last_iter = it
            pos_err, ori_err, e6 = self._errors(q, target_pose)
            if pos_err <= self.position_tol_m and ori_err <= self.orientation_tol_rad:
                if self.enforce_joint_limits and (
                    np.any(q < self.joint_lower_rad - 1e-9)
                    or np.any(q > self.joint_upper_rad + 1e-9)
                ):
                    return IKResult(
                        success=False,
                        q=q.copy(),
                        position_error_m=pos_err,
                        orientation_error_rad=ori_err,
                        iterations=it,
                        reason="joint_limit_violation",
                    )
                return IKResult(
                    success=True,
                    q=q.copy(),
                    position_error_m=pos_err,
                    orientation_error_rad=ori_err,
                    iterations=it,
                    reason="converged",
                )

            J = self.model.geometric_jacobian(q)
            # Damped least squares: dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
            jjt = J @ J.T
            a = jjt + lam2 * np.eye(6, dtype=float)
            try:
                dq = J.T @ np.linalg.solve(a, e6)
            except np.linalg.LinAlgError:
                return IKResult(
                    success=False,
                    q=q.copy(),
                    position_error_m=pos_err,
                    orientation_error_rad=ori_err,
                    iterations=it,
                    reason="singular_dls_system",
                )
            q = self._clamp_q(q + self.step_scale * dq)

        pos_err, ori_err, _ = self._errors(q, target_pose)
        reason = "max_iterations"
        if pos_err > self.position_tol_m and ori_err > self.orientation_tol_rad:
            reason = "max_iterations_pose_and_orientation"
        elif pos_err > self.position_tol_m:
            reason = "max_iterations_position"
        elif ori_err > self.orientation_tol_rad:
            reason = "max_iterations_orientation"
        return IKResult(
            success=False,
            q=q.copy(),
            position_error_m=pos_err,
            orientation_error_rad=ori_err,
            iterations=last_iter,
            reason=reason,
        )
