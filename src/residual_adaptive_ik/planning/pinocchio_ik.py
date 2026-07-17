# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Pinocchio-backed classical IK for the dexterity / dexterous-workspace gate.

Why Pinocchio (spec.md Phase 2 — dexterous-workspace gate)
----------------------------------------------------------
Plain NumPy DLS is a weak *completeness* oracle for 6-DOF orientation: it often
stalls on genuinely reachable pad-facing poses, so using it to skip targets
would dishonestly inflate the PLAN_OK gate. Pinocchio provides analytic
rigid-body FK / Jacobians on the same MyCobot URDF and is available under the
host Isaac Sim Python (verified). That makes multi-seed DLS with Pinocchio
Jacobians the best *classical, deterministic* library we can use here without
violating the ``q_final = q_ik + clamp(Δq)`` architecture mandate.

Optional dependency: ``import pinocchio`` — when unavailable (CI container),
callers fall back to the NumPy / FK-pool oracle in ``dexterity.py``.

Units: meters / radians. Quaternion convention: ``(w, x, y, z)``.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import (
    DEFAULT_EE_LINK,
    DEFAULT_REVOLUTE_JOINT_NAMES,
    resolve_urdf_path,
)


def pinocchio_available() -> bool:
    """True when the ``pinocchio`` package can be imported."""
    try:
        import pinocchio  # noqa: F401
    except Exception:
        return False
    return True


@dataclass(frozen=True)
class PinocchioIKResult:
    """Outcome of one Pinocchio multi-seed IK solve (SI units)."""

    success: bool
    q: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    reason: str


@lru_cache(maxsize=2)
def _load_pinocchio_model(
    urdf_path: str,
    ee_link: str,
) -> tuple[object, object, int, list[int]]:
    """Cached ``(model, data, ee_frame_id, q_indices)`` for the MyCobot URDF."""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    if not model.existFrame(ee_link):
        # Some prepared URDFs expose the EE as a body/link frame name.
        raise RuntimeError(f"Pinocchio model has no frame named {ee_link!r}")
    ee_id = model.getFrameId(ee_link)
    # Map our 6 revolute names → Pinocchio joint indices (configuration block).
    q_indices: list[int] = []
    for name in DEFAULT_REVOLUTE_JOINT_NAMES:
        if not model.existJointName(name):
            raise RuntimeError(f"Pinocchio model missing joint {name!r}")
        jid = model.getJointId(name)
        idx_q = model.joints[jid].idx_q
        q_indices.append(int(idx_q))
    return model, data, int(ee_id), q_indices


def _quat_wxyz_to_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """Unit quaternion (wxyz) → 3×3 rotation matrix."""
    w, x, y, z = np.asarray(quat_wxyz, dtype=float).reshape(4)
    n = float(np.linalg.norm([w, x, y, z]))
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _orientation_error_rad(R_cur: np.ndarray, R_des: np.ndarray) -> tuple[float, np.ndarray]:
    """Angle-axis orientation error (rad) and the 3-vector used in the DLS step."""
    import pinocchio as pin

    R_err = R_cur.T @ R_des
    # log3 returns the rotation vector in so(3); magnitude is the angle.
    w = pin.log3(R_err)
    ang = float(np.linalg.norm(w))
    return ang, w


def solve_pose_ik(
    target_position_m: np.ndarray,
    target_quaternion_wxyz: np.ndarray,
    *,
    seed_q: np.ndarray | None = None,
    urdf_path: Path | str | None = None,
    ee_link: str = DEFAULT_EE_LINK,
    max_iterations: int = 200,
    damping: float = 1e-2,
    step_scale: float = 0.5,
    position_tol_m: float = 0.006,
    orientation_tol_rad: float = 0.35,
    joint_lower_rad: np.ndarray | None = None,
    joint_upper_rad: np.ndarray | None = None,
) -> PinocchioIKResult:
    """Solve 6-DOF pose IK with Pinocchio FK/Jacobian DLS (one seed).

    Returns ``success=True`` only when both position and orientation tolerances
    are met and the solution respects joint limits.
    """
    import pinocchio as pin

    urdf = str(resolve_urdf_path(urdf_path))
    model, data, ee_id, q_idx = _load_pinocchio_model(urdf, ee_link)
    lo, hi = (
        (np.asarray(joint_lower_rad, dtype=float).reshape(6),
         np.asarray(joint_upper_rad, dtype=float).reshape(6))
        if joint_lower_rad is not None and joint_upper_rad is not None
        else load_joint_limits_rad()
    )
    q_pin = pin.neutral(model)
    if seed_q is None:
        q6 = 0.5 * (lo + hi)
    else:
        q6 = np.clip(np.asarray(seed_q, dtype=float).reshape(6), lo, hi)
    for i, iq in enumerate(q_idx):
        q_pin[iq] = q6[i]

    tgt_p = np.asarray(target_position_m, dtype=float).reshape(3)
    R_des = _quat_wxyz_to_matrix(target_quaternion_wxyz)
    pos_err = float("inf")
    ori_err = float("inf")
    last_it = 0

    for it in range(1, int(max_iterations) + 1):
        last_it = it
        pin.forwardKinematics(model, data, q_pin)
        pin.updateFramePlacements(model, data)
        oMf = data.oMf[ee_id]
        pos_err = float(np.linalg.norm(oMf.translation - tgt_p))
        ori_err, w_err = _orientation_error_rad(oMf.rotation, R_des)
        if pos_err <= position_tol_m and ori_err <= orientation_tol_rad:
            q_out = np.array([q_pin[iq] for iq in q_idx], dtype=float)
            if np.any(q_out < lo - 1e-9) or np.any(q_out > hi + 1e-9):
                return PinocchioIKResult(
                    False, q_out, pos_err, ori_err, it, "joint_limit_violation"
                )
            return PinocchioIKResult(True, q_out, pos_err, ori_err, it, "converged")

        J = pin.computeFrameJacobian(
            model, data, q_pin, ee_id, pin.LOCAL_WORLD_ALIGNED
        )
        # J is 6×nv; keep only our 6 revolute columns.
        J6 = J[:, q_idx]
        e6 = np.zeros(6, dtype=float)
        e6[:3] = tgt_p - oMf.translation
        # World-aligned angular error: rotate the body log into world.
        e6[3:] = oMf.rotation @ w_err
        lam2 = float(damping) * float(damping)
        a = J6 @ J6.T + lam2 * np.eye(6)
        try:
            dq6 = J6.T @ np.linalg.solve(a, e6)
        except np.linalg.LinAlgError:
            q_out = np.array([q_pin[iq] for iq in q_idx], dtype=float)
            return PinocchioIKResult(
                False, q_out, pos_err, ori_err, it, "singular_dls_system"
            )
        q6 = np.clip(
            np.array([q_pin[iq] for iq in q_idx], dtype=float)
            + float(step_scale) * dq6,
            lo,
            hi,
        )
        for i, iq in enumerate(q_idx):
            q_pin[iq] = q6[i]

    q_out = np.array([q_pin[iq] for iq in q_idx], dtype=float)
    return PinocchioIKResult(
        False, q_out, pos_err, ori_err, last_it, "max_iterations"
    )


def multi_seed_pose_reachable(
    target_position_m: np.ndarray,
    target_quaternion_wxyz: np.ndarray,
    *,
    seeds: list[np.ndarray],
    urdf_path: Path | str | None = None,
    position_tol_m: float = 0.006,
    orientation_tol_rad: float = 0.35,
) -> PinocchioIKResult:
    """Try several seeds; return the first success or the best failure."""
    best = PinocchioIKResult(
        False,
        np.full(6, np.nan),
        float("inf"),
        float("inf"),
        0,
        "no_seeds",
    )
    for seed in seeds:
        res = solve_pose_ik(
            target_position_m,
            target_quaternion_wxyz,
            seed_q=seed,
            urdf_path=urdf_path,
            position_tol_m=position_tol_m,
            orientation_tol_rad=orientation_tol_rad,
        )
        if res.success:
            return res
        if res.position_error_m < best.position_error_m or (
            abs(res.position_error_m - best.position_error_m) < 1e-9
            and res.orientation_error_rad < best.orientation_error_rad
        ):
            best = res
    return best
