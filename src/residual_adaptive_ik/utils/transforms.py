# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""SE(3) / quaternion helpers with explicit units.

Pose convention matches ``fk.Pose``: position in meters, quaternion ``(w, x, y, z)``.
Orientation error uses the rotation-vector magnitude (radians). See ``spec.md`` Phase 1.
"""
from __future__ import annotations

import numpy as np


def quaternion_wxyz_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion ``(w, x, y, z)``."""
    q = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("zero-norm quaternion")
    return q / n


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of ``(w, x, y, z)`` (inverse for unit quaternions)."""
    q = quaternion_wxyz_normalize(q)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product ``q1 ⊗ q2`` for ``(w, x, y, z)``."""
    w1, x1, y1, z1 = quaternion_wxyz_normalize(q1)
    w2, x2, y2, z2 = quaternion_wxyz_normalize(q2)
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Unit quaternion ``(w, x, y, z)`` → 3×3 rotation matrix."""
    w, x, y, z = quaternion_wxyz_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Log map: 3×3 rotation → rotation vector (axis × angle, radians)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos_theta = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-12:
        return np.zeros(3, dtype=float)
    if abs(theta - np.pi) < 1e-6:
        # Near 180°: extract axis from diagonal of R
        axis = np.sqrt(np.maximum(np.diag(R) + 1.0, 0.0) / 2.0)
        # Resolve signs from off-diagonals
        if R[2, 1] < R[1, 2]:
            axis[0] = -axis[0]
        if R[0, 2] < R[2, 0]:
            axis[1] = -axis[1]
        if R[1, 0] < R[0, 1]:
            axis[2] = -axis[2]
        n = np.linalg.norm(axis)
        if n < 1e-12:
            return np.array([theta, 0.0, 0.0], dtype=float)
        return axis / n * theta
    w = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=float,
    )
    return w * (theta / (2.0 * np.sin(theta)))


def orientation_error_rad(q_current_wxyz: np.ndarray, q_target_wxyz: np.ndarray) -> np.ndarray:
    """Rotation-vector error (radians) taking ``current`` toward ``target``.

    ``R_err = R_target @ R_current.T`` expressed as a rotation vector in the
    base frame. Magnitude is the geodesic angle on SO(3).
    """
    R_cur = quaternion_to_rotation_matrix(q_current_wxyz)
    R_tgt = quaternion_to_rotation_matrix(q_target_wxyz)
    R_err = R_tgt @ R_cur.T
    return rotation_matrix_to_rotvec(R_err)


def pose_error_6(
    position_m: np.ndarray,
    quaternion_wxyz: np.ndarray,
    target_position_m: np.ndarray,
    target_quaternion_wxyz: np.ndarray,
) -> np.ndarray:
    """6-vector pose error ``[Δp (m); ω (rad)]`` for DLS IK."""
    dp = np.asarray(target_position_m, dtype=float).reshape(3) - np.asarray(
        position_m, dtype=float
    ).reshape(3)
    do = orientation_error_rad(quaternion_wxyz, target_quaternion_wxyz)
    return np.concatenate([dp, do])


def position_error_norm_m(a: np.ndarray, b: np.ndarray) -> float:
    """Euclidean position error in meters."""
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def orientation_error_magnitude_rad(q_a_wxyz: np.ndarray, q_b_wxyz: np.ndarray) -> float:
    """Geodesic orientation error magnitude in radians (quaternion sign-invariant)."""
    return float(np.linalg.norm(orientation_error_rad(q_a_wxyz, q_b_wxyz)))
