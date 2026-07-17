# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Oriented tip-face contact geometry for Phase 2 marker approaches.

Why
---
Planning the entire path with tip/flange collision spheres omitted lets the EE
**side** sweep through the volumetric marker while the tip still lands on the
surface. Industry practice (approach–retreat) is:

1. Keep tip spheres **on** until a short contact standoff along the sphere
   normal (tool +Z facing the sphere).
2. Omit tip spheres only for a **bounded axial nudge** onto the pierce point.

Units: meters / radians. Tool +Z is the URDF EE ``joint6_flange`` local Z axis
(pad outward). Contact orientation aligns tool +Z with the outward surface
normal (from pierce toward tip / away from center), so the pad faces the sphere.
See ``spec.md`` Phase 2 / tip-face contact.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from residual_adaptive_ik.kinematics.urdf_model import (
    rotation_matrix_to_quaternion_wxyz,
)
from residual_adaptive_ik.utils.transforms import quaternion_to_rotation_matrix


@dataclass(frozen=True)
class ContactApproach:
    """Oriented standoff → pierce segment for tip-face contact (meters / wxyz)."""

    normal_outward: np.ndarray  # unit, pierce → away from center (pad faces sphere)
    pierce_position_m: np.ndarray
    standoff_position_m: np.ndarray
    quaternion_wxyz: np.ndarray
    standoff_m: float
    nudge_m: float


def tool_axis_from_quaternion(
    quaternion_wxyz: np.ndarray,
    *,
    tool_axis_local: np.ndarray | None = None,
) -> np.ndarray:
    """Return tool-axis unit vector in the base frame (default local +Z)."""
    R = quaternion_to_rotation_matrix(quaternion_wxyz)
    axis_local = (
        np.array([0.0, 0.0, 1.0], dtype=float)
        if tool_axis_local is None
        else np.asarray(tool_axis_local, dtype=float).reshape(3)
    )
    n = float(np.linalg.norm(axis_local))
    if n < 1e-12:
        axis_local = np.array([0.0, 0.0, 1.0], dtype=float)
        n = 1.0
    axis_local = axis_local / n
    v = R @ axis_local
    vn = float(np.linalg.norm(v))
    if vn < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return v / vn


def tool_axis_alignment_error_rad(
    quaternion_wxyz: np.ndarray,
    desired_axis_base: np.ndarray,
    *,
    tool_axis_local: np.ndarray | None = None,
) -> float:
    """Angle (rad) between tool axis and desired base-frame direction."""
    u = tool_axis_from_quaternion(
        quaternion_wxyz, tool_axis_local=tool_axis_local
    )
    d = np.asarray(desired_axis_base, dtype=float).reshape(3)
    dn = float(np.linalg.norm(d))
    if dn < 1e-12:
        return float(np.pi)
    d = d / dn
    c = float(np.clip(np.dot(u, d), -1.0, 1.0))
    return float(np.arccos(c))


def _orthonormal_basis_from_z(
    z_axis: np.ndarray,
    *,
    x_hint: np.ndarray | None = None,
) -> np.ndarray:
    """Build R with columns (x, y, z) where ``z`` is unit and faces the sphere."""
    z = np.asarray(z_axis, dtype=float).reshape(3)
    zn = float(np.linalg.norm(z))
    if zn < 1e-12:
        z = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        z = z / zn
    hint = (
        np.array([1.0, 0.0, 0.0], dtype=float)
        if x_hint is None
        else np.asarray(x_hint, dtype=float).reshape(3)
    )
    # Project hint onto plane ⊥ z; fall back if nearly parallel.
    x = hint - float(np.dot(hint, z)) * z
    xn = float(np.linalg.norm(x))
    if xn < 1e-8:
        hint2 = np.array([0.0, 1.0, 0.0], dtype=float)
        x = hint2 - float(np.dot(hint2, z)) * z
        xn = float(np.linalg.norm(x))
        if xn < 1e-8:
            hint3 = np.array([0.0, 0.0, 1.0], dtype=float)
            x = hint3 - float(np.dot(hint3, z)) * z
            xn = float(np.linalg.norm(x))
    x = x / max(xn, 1e-12)
    y = np.cross(z, x)
    yn = float(np.linalg.norm(y))
    y = y / max(yn, 1e-12)
    # Re-orthogonalize x in case of drift.
    x = np.cross(y, z)
    return np.column_stack((x, y, z))


def quaternion_facing_sphere(
    tool_z_direction: np.ndarray,
    *,
    x_hint: np.ndarray | None = None,
) -> np.ndarray:
    """Quaternion (wxyz) with tool +Z along ``tool_z_direction`` (this planning
    frame's +Z; opposite the URDF/FK tool +Z — see ``build_sphere_contact_approach``)."""
    R = _orthonormal_basis_from_z(tool_z_direction, x_hint=x_hint)
    return rotation_matrix_to_quaternion_wxyz(R)


def contact_orientation_cone(
    normal_outward: np.ndarray,
    *,
    cone_max_rad: float = 0.30,
    n_tilts: int = 2,
    n_azimuths: int = 4,
    x_hint: np.ndarray | None = None,
    current_quaternion_wxyz: np.ndarray | None = None,
) -> list[np.ndarray]:
    r"""Bounded cone of pad-facing contact orientations (wxyz), exact axis first.

    Why (spec.md Phase 2 — orientation cone for edge targets)
    ---------------------------------------------------------
    Commanding the *single* exact outward-normal orientation
    (``quaternion_facing_sphere(normal)``) is often ``IK_FAIL`` for targets near
    the reach envelope: the wrist cannot achieve that one orientation there.
    Allowing a **small cone** of tool-axis directions around the outward normal
    gives cuRobo alternative wrist solutions while staying an honest tip-face
    contact.

    Honesty bound
    -------------
    Every returned orientation keeps tool +Z within ``cone_max_rad`` of the exact
    outward normal, so the achieved contact still satisfies the signed tip-face
    gate (``classify_tip_contact`` requires ``axis_out ≤``
    ``TARGET_MARKER_TOOL_AXIS_TOL_RAD`` ≈ 15°). Keep ``cone_max_rad`` ≤ ~0.5 rad
    so a small servo/solve error cannot push a cone edge past the gate. This is
    **not** a way to accept side/through contacts — the lateral / penetration
    checks still apply.

    Ordering
    --------
    The exact outward normal is returned **first** (preferred), then rings of
    increasing tilt (``cone_max_rad/n_tilts … cone_max_rad``) each sampled at
    ``n_azimuths`` azimuths. Deterministic. Units: radians.
    """
    normal = np.asarray(normal_outward, dtype=float).reshape(3)
    nn = float(np.linalg.norm(normal))
    normal = normal / nn if nn > 1e-12 else np.array([0.0, 0.0, 1.0], dtype=float)
    if x_hint is None and current_quaternion_wxyz is not None:
        R0 = quaternion_to_rotation_matrix(current_quaternion_wxyz)
        x_hint = R0[:, 0]
    quats: list[np.ndarray] = [quaternion_facing_sphere(normal, x_hint=x_hint)]
    cone = float(cone_max_rad)
    if cone <= 1e-6 or int(n_tilts) <= 0 or int(n_azimuths) <= 0:
        return quats
    R = _orthonormal_basis_from_z(normal, x_hint=x_hint)
    ex, ey = R[:, 0], R[:, 1]
    for ti in range(1, int(n_tilts) + 1):
        tilt = cone * (float(ti) / float(n_tilts))
        ct, st = float(np.cos(tilt)), float(np.sin(tilt))
        for k in range(int(n_azimuths)):
            az = 2.0 * np.pi * float(k) / float(n_azimuths)
            direction = ct * normal + st * (float(np.cos(az)) * ex + float(np.sin(az)) * ey)
            quats.append(quaternion_facing_sphere(direction, x_hint=ex))
    return quats


def build_sphere_contact_approach(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    *,
    standoff_m: float = 0.015,
    nudge_m: float | None = None,
    x_hint: np.ndarray | None = None,
    current_quaternion_wxyz: np.ndarray | None = None,
) -> ContactApproach:
    """Build oriented standoff + pierce on the approach side of the sphere.

    The outward normal at the pierce points from the center toward the tip
    start (near hemisphere); ``standoff`` sits ``standoff_m`` further out along
    that ray and ``pierce`` is on the surface (``nudge_m ≈ standoff_m``).

    Tool +Z is commanded along the **outward** normal (``quaternion_facing_sphere``
    convention). Empirically this is the orientation cuRobo can actually reach
    for a near-side approach — commanding the *inward* sign instead makes the
    contact-approach IK fail (``MotionGenStatus.IK_FAIL``). NOTE: this planning
    frame's +Z is **opposite** the URDF/FK tool +Z, so the *achieved* pose reads
    as +Z-toward-center when measured through ``forward_kinematics`` — which is
    exactly what the signed tip-face gate (``classify_tip_contact``) requires.
    See ``docs/phase2_geometry.md`` (frame-convention note).
    """
    start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    radius = float(sphere_radius_m)
    stand = max(1e-4, float(standoff_m))
    nudge = float(stand if nudge_m is None else nudge_m)
    nudge = float(np.clip(nudge, 1e-4, stand + 1e-6))

    delta = start - center  # outward from center toward tip
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        normal = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        normal = delta / dist

    pierce = center + radius * normal
    standoff = pierce + stand * normal

    if x_hint is None and current_quaternion_wxyz is not None:
        R = quaternion_to_rotation_matrix(current_quaternion_wxyz)
        x_hint = R[:, 0]
    # Command +Z along the outward normal (reachable orientation for cuRobo);
    # the URDF/FK tool +Z is opposite, so the achieved contact reads inward.
    quat = quaternion_facing_sphere(normal, x_hint=x_hint)

    return ContactApproach(
        normal_outward=normal,
        pierce_position_m=pierce,
        standoff_position_m=standoff,
        quaternion_wxyz=quat,
        standoff_m=stand,
        nudge_m=nudge,
    )


def validate_axial_contact_segment(
    start_m: np.ndarray,
    end_m: np.ndarray,
    normal_outward: np.ndarray,
    *,
    max_length_m: float,
    lateral_tol_m: float = 0.002,
) -> tuple[bool, str]:
    """Return (ok, reason) for a short axial standoff→pierce segment."""
    a = np.asarray(start_m, dtype=float).reshape(3)
    b = np.asarray(end_m, dtype=float).reshape(3)
    n = np.asarray(normal_outward, dtype=float).reshape(3)
    nn = float(np.linalg.norm(n))
    if nn < 1e-12:
        return False, "bad_normal"
    n = n / nn
    delta = b - a
    length = float(np.linalg.norm(delta))
    if length > float(max_length_m) + 1e-6:
        return False, f"too_long:{length:.4f}>{max_length_m:.4f}"
    # Progress should be toward the sphere (against outward normal).
    axial = float(np.dot(delta, n))
    if axial > 1e-4:
        return False, f"wrong_direction:{axial:.4f}"
    lateral = float(np.linalg.norm(delta - axial * n))
    if lateral > float(lateral_tol_m) + 1e-6:
        return False, f"not_axial:lateral={lateral:.4f}"
    return True, "ok"
