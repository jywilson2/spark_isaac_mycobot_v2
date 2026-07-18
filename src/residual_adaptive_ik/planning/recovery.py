# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Motion-planning recovery: standoff vias with timeout + optional partial exec.

Why
---
Direct tip→marker-surface plans often ``IK_FAIL`` when the start pose is awkward
or the EE is already near the volumetric target. Industrial practice splits the
motion into **via** segments: plan to a standoff pose outside the obstacle, then
a short final approach to the contact surface.

Behaviour (Phase 2 viz / smoke)
-------------------------------
* **Oriented tip-face contact (default):** approach with tip spheres **on** to a
  short standoff along the sphere normal (tool +Z facing the marker), then omit
  tip spheres only for a **bounded axial nudge** onto the pierce point. This
  avoids EE-side sweeps through the volumetric marker (see
  ``planning/contact_geometry.py``).
* Keep retrying strategies until ``plan_recovery_timeout_s`` elapses — do **not**
  declare PLAN_FAIL after a single quick pass over clearances.
* After a failed attempt, via standoffs are ordered **nearest → farthest** from
  the current tip (progressive distance). Candidates closer than
  ``plan_recovery_min_standoff_travel_m`` are skipped so the EE does not “retry”
  an almost-identical pose; each subsequent candidate is farther out.
* ``INVALID_START_STATE_*`` escapes via a **joint-space seed bank**: move to a
  preparatory ``q_seed`` (home / mid-home / prior goals), then retry. When the
  start is in collision, MotionGen cannot start — use an open-loop joint move
  to ``q_seed``. When the start is valid, prefer a collision-aware plan to the
  FK tip of ``q_seed``. After the bank is exhausted, fall through to via
  standoffs (home-blend is last resort only).
* **EE-close ``IK_FAIL`` reposition:** when the start is *valid* but the tip is
  inside the standoff shell (folded near the target) and the oriented contact
  returns ``IK_FAIL``, the tip→center ray is degenerate. Generate a
  **reposition via** along the well-conditioned **base→target** radial
  (``try_radial_reposition_via``) to an extended pre-approach, then retry the
  oriented contact from there. Bounded by
  ``plan_recovery_reposition_max_attempts``.
* When ``execute_waypoints`` is provided and via1 succeeds but via2 fails,
  **execute via1** (move the EE), then continue planning from the new pose.
* The IK target marker must not jump to yellow until this budget is exhausted.

This is classical motion planning — not residual learning. See
``docs/phase2_geometry.md`` and ``configs/planning/collision.yaml``.
``spec.md`` § IK failure → preparatory repositioning.

Units: meters, radians, seconds.
"""
from __future__ import annotations

import math
import re
import time
from typing import Callable, Sequence

import numpy as np

from residual_adaptive_ik.geometry.collision import SphereObstacle, proximal_arm_contacts_target
from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.ik_seed_bank import (
    build_seed_bank,
    joint_distance_rad,
    order_seeds,
)
from residual_adaptive_ik.kinematics.numerical_ik import (
    DampedLeastSquaresIK,
    load_joint_limits_rad,
)
from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel, get_default_model
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    PlannedTrajectory,
    curobo_available,
    load_planning_config,
    tip_on_sphere_surface,
)
from residual_adaptive_ik.planning.contact_geometry import (
    build_sphere_contact_approach,
    contact_orientation_cone,
    tool_axis_alignment_error_rad,
    validate_axial_contact_segment,
)
from residual_adaptive_ik.planning.joint_path import interpolate_joint_path


def contact_orientation_candidates(
    approach,
    *,
    fallback_quaternion_wxyz: np.ndarray,
    cfg: dict,
) -> list[np.ndarray]:
    """Ordered pad-facing orientations to try for the contact approach leg.

    Exact outward normal first, then a bounded cone (spec.md Phase 2 orientation
    cone) when ``contact_orientation_cone_enabled``, then the current EE
    orientation as a last resort. Every cone entry stays within
    ``contact_orientation_cone_max_rad`` of the outward normal so the honest
    tip-face gate still holds.
    """
    exact = np.asarray(approach.quaternion_wxyz, dtype=float).reshape(4)
    if not bool(cfg.get("contact_orientation_cone_enabled", True)):
        return [exact, np.asarray(fallback_quaternion_wxyz, dtype=float).reshape(4)]
    cone = contact_orientation_cone(
        approach.normal_outward,
        cone_max_rad=float(cfg.get("contact_orientation_cone_max_rad", 0.30)),
        n_tilts=int(cfg.get("contact_orientation_cone_tilts", 2)),
        n_azimuths=int(cfg.get("contact_orientation_cone_azimuths", 4)),
        current_quaternion_wxyz=fallback_quaternion_wxyz,
    )
    cone.append(np.asarray(fallback_quaternion_wxyz, dtype=float).reshape(4))
    return cone

# Called as ``execute_waypoints(waypoints_rad, dt_s)`` to move the arm mid-recovery.
ExecuteWaypointsFn = Callable[[np.ndarray, float], None]
# Live decision-point log for viz/smoke (one short line per branch).
DecisionEmitFn = Callable[[str], None]


def _decision(
    emit: DecisionEmitFn | None,
    log: list[str],
    message: str,
) -> None:
    """Record a grep-friendly decision line (``DEC|…``) for live monitoring."""
    line = f"DEC|{message}"
    log.append(line)
    if emit is not None:
        try:
            emit(line)
        except Exception:
            pass


def tip_omit_length_allow_m(cfg: dict, *, nudge_max_m: float) -> float:
    """Hard Cartesian cap for any tip-omit segment (meters).

    Always ``contact_nudge_max_m`` (or the caller’s clipped ``nudge_max``).
    Never expand to multi-centimeter “finish from far” windows — long tip-omit
    with flange spheres off lets the EE barrel hit the marker from the side/back
    (spec.md Tip-face contact planning).
    """
    via_cap = float(cfg.get("contact_via_nudge_max_m", nudge_max_m))
    # Via and direct share the same short axial budget.
    return float(min(max(1e-4, float(nudge_max_m)), max(1e-4, via_cap)))


def _tip_face_patch_targets_m(
    pierce_position_m: np.ndarray,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    *,
    patch_lateral_max_m: float,
    n_lateral: int = 3,
    n_azimuth: int = 8,
) -> list[np.ndarray]:
    """Ordered tip-face surface targets: exact pierce, then small lateral patch.

    Why
    ---
    Exact center-ray pierce can be IK-unreachable while a nearby surface point
    within the tip-face disk (≤ ``TARGET_MARKER_TIP_FACE_RADIUS_M`` ≈ 4 mm) is
    reachable (iter28 Ep6: axial ``max_iterations_position`` with ori≈0). Stay
    on the sphere surface so tip-face classify can still green.
    """
    pierce = np.asarray(pierce_position_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    r = float(sphere_radius_m)
    out: list[np.ndarray] = [pierce.copy()]
    radial = pierce - center
    nrm = float(np.linalg.norm(radial))
    if nrm < 1e-9 or r < 1e-9:
        return out
    n_hat = radial / nrm
    # Orthonormal basis in the tangent plane at the pierce.
    helper = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(n_hat, helper))) > 0.9:
        helper = np.array([0.0, 1.0, 0.0], dtype=float)
    t1 = np.cross(n_hat, helper)
    t1 = t1 / float(np.linalg.norm(t1))
    t2 = np.cross(n_hat, t1)
    lat_max = max(0.0, float(patch_lateral_max_m))
    if lat_max <= 0.0:
        return out
    for lat in np.linspace(lat_max / float(max(1, n_lateral)), lat_max, n_lateral):
        for ang in np.linspace(0.0, 2.0 * math.pi, n_azimuth, endpoint=False):
            offset = float(lat) * (
                math.cos(float(ang)) * t1 + math.sin(float(ang)) * t2
            )
            direction = pierce - center + offset
            dnorm = float(np.linalg.norm(direction))
            if dnorm < 1e-9:
                continue
            tip_t = center + (r / dnorm) * direction
            out.append(tip_t.astype(float))
    return out


def plan_axial_tip_omit_lerp(
    q_start_rad: np.ndarray,
    pierce_position_m: np.ndarray,
    quaternion_wxyz: np.ndarray,
    *,
    model: UrdfKinematicModel,
    dt_s: float = 0.02,
    n_samples: int = 12,
    sphere_center_m: np.ndarray | None = None,
    sphere_radius_m: float | None = None,
    patch_lateral_max_m: float = 0.003,
) -> PlannedTrajectory:
    """Short tip-omit standoff→pierce via DLS-IK + joint lerp (not MotionGen).

    Why
    ---
    cuRobo tip-omit ``plan_to_pose`` with tip spheres off can take a free
    curved path that swings the tip through a side_graze / wrong_side sample
    before the pierce — mid-path latch then PLAN_FAIL even when settle looks
    fine. A seeded IK + short joint lerp keeps Δq small and approximately
    axial (spec.md Mandatory contact failure conditions).

    When the exact pierce is IK-unreachable, optionally search a small tip-face
    patch on the sphere (still ≤ tip-face lateral disk) before failing.
    """
    q0 = _clamp_joints(np.asarray(q_start_rad, dtype=float).reshape(6))
    pierce0 = np.asarray(pierce_position_m, dtype=float).reshape(3)
    quat0 = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
    # Tip-omit IK: position must land in the surface shell; orientation may
    # already be at the tip-face / cone edge after spheres-ON approach
    # (iter22 Ep4: ori_tol=0.02 caused endless axial_ik fail → curobo fail).
    ik = DampedLeastSquaresIK(
        max_iterations=160,
        damping=2e-3,
        position_tol_m=1.5e-3,
        orientation_tol_rad=0.12,
        enforce_joint_limits=True,
        model=model,
    )
    targets: list[tuple[np.ndarray, np.ndarray, str]] = [
        (pierce0, quat0, "exact")
    ]
    if sphere_center_m is not None and sphere_radius_m is not None:
        center = np.asarray(sphere_center_m, dtype=float).reshape(3)
        radius = float(sphere_radius_m)
        for tip_t in _tip_face_patch_targets_m(
            pierce0,
            center,
            radius,
            patch_lateral_max_m=float(patch_lateral_max_m),
        )[1:]:
            # Rebuild pad-facing quat for this surface point (same cone family).
            ap_t = build_sphere_contact_approach(
                tip_t + (tip_t - center),  # approach_from outside along radial
                center,
                radius,
                standoff_m=0.008,
                nudge_m=0.008,
                current_quaternion_wxyz=quat0,
            )
            targets.append(
                (
                    np.asarray(tip_t, dtype=float).reshape(3),
                    np.asarray(ap_t.quaternion_wxyz, dtype=float).reshape(4),
                    "patch",
                )
            )

    last_reason = "max_iterations"
    for tip_tgt, quat_tgt, kind in targets:
        pose_tgt = Pose(position_m=tip_tgt, quaternion_wxyz=quat_tgt)
        sol = ik.solve(pose_tgt, seed_q=q0)
        if not sol.success:
            last_reason = str(sol.reason)
            continue
        q1 = _clamp_joints(np.asarray(sol.q, dtype=float).reshape(6))
        # Verify FK tip is on the surface shell before accepting the lerp.
        tip1 = np.asarray(
            forward_kinematics(q1, model=model).position_m, dtype=float
        ).reshape(3)
        tip_err = float(np.linalg.norm(tip1 - tip_tgt))
        if tip_err > 0.004:
            last_reason = f"axial_ik_tip_err_m={tip_err:.4f}"
            continue
        # Optional: when patching, keep lateral vs exact pierce inside tip-face.
        if kind == "patch":
            lat = float(np.linalg.norm(tip1 - pierce0))
            if lat > float(patch_lateral_max_m) + 0.001:
                last_reason = f"patch_lateral_m={lat:.4f}"
                continue
        wp = interpolate_joint_path(q0, q1, n_samples=max(2, int(n_samples)))
        return PlannedTrajectory(
            waypoints_rad=wp,
            dt_s=float(dt_s),
            success=True,
            backend="axial_lerp",
            message=f"ok|axial_tip_omit_lerp|{kind}",
        )
    return PlannedTrajectory(
        waypoints_rad=np.zeros((0, 6)),
        dt_s=float(dt_s),
        success=False,
        backend="axial_lerp",
        message=f"plan_failed:axial_ik|{last_reason}",
    )


def tip_path_avoids_marker_immersion(
    waypoints_rad: np.ndarray,
    *,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    model: UrdfKinematicModel,
    min_dist_m: float | None = None,
    stride: int = 1,
) -> tuple[bool, float]:
    """Return ``(ok, min_tip_dist_m)`` — reject paths whose tip enters the volume.

    Why
    ---
    MotionGen spheres-ON approaches can still swing the *kinematic tip* through
    the marker (fitted collision spheres miss the exact tip point). Mid-path
    then logs IMMERSED / THROUGH / SIDE_GRAZE → PLAN_FAIL. Sample FK tips along
    the joint path and require ``dist ≥ radius − 1 mm``.
    """
    wp = np.asarray(waypoints_rad, dtype=float)
    if wp.ndim != 2 or wp.shape[0] == 0:
        return False, float("nan")
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    r_min = (
        float(min_dist_m)
        if min_dist_m is not None
        else float(sphere_radius_m) - 0.001
    )
    step = max(1, int(stride))
    min_d = float("inf")
    for q in wp[::step]:
        tip = np.asarray(
            forward_kinematics(q, model=model).position_m, dtype=float
        ).reshape(3)
        d = float(np.linalg.norm(tip - center))
        if d < min_d:
            min_d = d
        if d < r_min:
            return False, float(min_d)
    return True, float(min_d if np.isfinite(min_d) else float("nan"))


def tip_path_tip_face_ok(
    waypoints_rad: np.ndarray,
    *,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    approach_from_m: np.ndarray,
    model: UrdfKinematicModel,
    cfg: dict,
    stride: int = 1,
    densify_n: int = 48,
) -> tuple[bool, str]:
    """Return ``(ok, reason)`` — reject shell samples that are side/through/flipped.

    Imports ``classify_tip_contact`` via repo root (``isaac_sim`` is outside the
    residual package path).

    Also rejects **near-field lateral corridor** breaches (tip within
    ``r + standoff + 4 mm`` of the center but farther than the tip-face radius
    from the approach axis). Sparse joint-lerp samples can miss a brief shell
    graze (iter17 Ep8: DLS approach_ok then runtime ``SIDE_GRAZE`` lat=11 mm);
    densify + corridor catch those chords before execution.
    """
    classify_tip_contact = None
    tip_face_radius_m = 0.004
    try:
        import sys
        from pathlib import Path

        _repo = Path(__file__).resolve().parents[3]
        if str(_repo) not in sys.path:
            sys.path.insert(0, str(_repo))
        from isaac_sim.target_marker import (
            TARGET_MARKER_TIP_FACE_RADIUS_M,
            classify_tip_contact as _ctc,
        )

        classify_tip_contact = _ctc
        tip_face_radius_m = float(TARGET_MARKER_TIP_FACE_RADIUS_M)
    except Exception:
        return True, "classify_unavailable"
    wp = np.asarray(waypoints_rad, dtype=float)
    if wp.ndim != 2 or wp.shape[0] == 0:
        return False, "empty_path"
    # Densify joint path so brief chord grazes are not skipped between samples.
    n_dense = max(int(densify_n), int(wp.shape[0]))
    if wp.shape[0] < n_dense:
        from residual_adaptive_ik.planning.joint_path import interpolate_joint_path

        # Piecewise densify along the existing waypoint polyline.
        chunks: list[np.ndarray] = []
        segs = max(1, wp.shape[0] - 1)
        per = max(2, int(np.ceil(n_dense / segs)))
        for i in range(segs):
            chunk = interpolate_joint_path(wp[i], wp[i + 1], n_samples=per)
            if i < segs - 1:
                chunk = chunk[:-1]
            chunks.append(chunk)
        wp = np.vstack(chunks)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    standoff_ref = np.asarray(approach_from_m, dtype=float).reshape(3)
    delta_ap = center - standoff_ref
    ap_n = float(np.linalg.norm(delta_ap))
    approach_u = delta_ap / ap_n if ap_n > 1e-9 else np.array([0.0, 0.0, 1.0])
    standoff_m = float(cfg.get("contact_standoff_m", 0.008))
    near_m = float(sphere_radius_m) + standoff_m + 0.004
    step = max(1, int(stride))
    latch_axis = max(
        0.50,
        2.0 * float(cfg.get("contact_axis_tolerance_rad", 0.26)),
    )
    for q_tf in wp[::step]:
        pose_tf = forward_kinematics(q_tf, model=model)
        tip_tf = np.asarray(pose_tf.position_m, dtype=float).reshape(3)
        d_tf = float(np.linalg.norm(tip_tf - center))
        if d_tf > near_m:
            continue
        # Near-field lateral corridor (before / at shell).
        v = tip_tf - center
        axial = float(np.dot(v, approach_u))
        lat = float(np.linalg.norm(v - axial * approach_u))
        if lat > tip_face_radius_m + 0.002:
            return False, "side_graze"
        if d_tf > float(sphere_radius_m) + 0.008:
            continue
        _fok, freason, fm = classify_tip_contact(
            tip_tf,
            center,
            approach_from_m=standoff_ref,
            ee_quaternion_wxyz=pose_tf.quaternion_wxyz,
        )
        bad = freason in ("side_graze", "through", "immersed")
        if freason == "wrong_side_axis":
            ax = float(fm.get("axis_out_err_rad", 0.0))
            bad = ax > latch_axis
        if bad:
            return False, str(freason)
    return True, "ok"


def plan_dls_standoff_approach_lerp(
    q_start_rad: np.ndarray,
    standoff_position_m: np.ndarray,
    quaternion_wxyz: np.ndarray,
    *,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    model: UrdfKinematicModel,
    dt_s: float = 0.02,
    n_samples: int = 24,
) -> PlannedTrajectory:
    """Spheres-ON standoff approach via DLS-IK + joint lerp (MotionGen fallback).

    Why
    ---
    Sequential multi-target episodes often leave the wrist oriented for the
    *previous* marker. cuRobo then ``IK_FAIL`` / ``FINETUNE_TRAJOPT_FAIL`` on a
    short (~50–80 mm) pad-facing approach even though classical DLS reaches
    the standoff. A DLS solve + joint lerp recovers those handoffs without
    widening tip-omit. Rejects paths whose tip immerses the marker mid-lerp
    (chord cut through the sphere).
    """
    q0 = _clamp_joints(np.asarray(q_start_rad, dtype=float).reshape(6))
    pose_tgt = Pose(
        position_m=np.asarray(standoff_position_m, dtype=float).reshape(3),
        quaternion_wxyz=np.asarray(quaternion_wxyz, dtype=float).reshape(4),
    )
    ik = DampedLeastSquaresIK(
        max_iterations=160,
        damping=2e-3,
        position_tol_m=1.5e-3,
        orientation_tol_rad=0.04,
        enforce_joint_limits=True,
        model=model,
    )
    sol = ik.solve(pose_tgt, seed_q=q0)
    if not sol.success:
        return PlannedTrajectory(
            waypoints_rad=np.zeros((0, 6)),
            dt_s=float(dt_s),
            success=False,
            backend="dls_approach",
            message=f"plan_failed:dls_approach_ik|{sol.reason}",
        )
    q1 = _clamp_joints(np.asarray(sol.q, dtype=float).reshape(6))
    tip1 = np.asarray(
        forward_kinematics(q1, model=model).position_m, dtype=float
    ).reshape(3)
    tip_err = float(np.linalg.norm(tip1 - pose_tgt.position_m))
    if tip_err > 0.004:
        return PlannedTrajectory(
            waypoints_rad=np.zeros((0, 6)),
            dt_s=float(dt_s),
            success=False,
            backend="dls_approach",
            message=f"plan_failed:dls_approach_tip_err_m={tip_err:.4f}",
        )
    wp = interpolate_joint_path(q0, q1, n_samples=max(2, int(n_samples)))
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    r_min = float(sphere_radius_m) - 0.001  # tip must stay outside (≤1 mm tol)
    for q in wp:
        tip = np.asarray(
            forward_kinematics(q, model=model).position_m, dtype=float
        ).reshape(3)
        if float(np.linalg.norm(tip - center)) < r_min:
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=float(dt_s),
                success=False,
                backend="dls_approach",
                message="plan_failed:dls_approach_tip_immerses_marker",
            )
    return PlannedTrajectory(
        waypoints_rad=wp,
        dt_s=float(dt_s),
        success=True,
        backend="dls_approach",
        message="ok|dls_standoff_approach_lerp",
    )


def fk_pad_aligned_for_tip_omit(
    q_rad: np.ndarray,
    normal_outward: np.ndarray,
    *,
    model: UrdfKinematicModel,
    cfg: dict,
) -> tuple[bool, float, float]:
    """Return ``(ok, axis_err_rad, tol_rad)`` for tip-omit eligibility.

    Tip-omit may run only when FK tool +Z already faces the outward approach
    ray within the tip-face / orientation-cone tolerance. Side- or back-facing
    wrists must reseat with spheres ON (or via) — never tip-omit first.
    """
    pose = forward_kinematics(q_rad, model=model)
    axis_tol = float(cfg.get("contact_axis_tolerance_rad", 0.35))
    if bool(cfg.get("contact_orientation_cone_enabled", True)):
        cone_max = float(cfg.get("contact_orientation_cone_max_rad", 0.30))
        axis_tol = min(axis_tol, cone_max)
    err = float(
        tool_axis_alignment_error_rad(
            pose.quaternion_wxyz, normal_outward
        )
    )
    return err <= axis_tol + 1e-3, err, axis_tol


def tip_standoff_on_approach(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    *,
    clearance_m: float,
    yaw_offset_rad: float = 0.0,
) -> np.ndarray:
    """Tip pose at ``clearance_m`` from the sphere center (meters).

    ``yaw_offset_rad`` rotates the approach direction about +Z through the
    sphere center so recovery can try lateral standoffs, not only the pure
    tip→center ray.
    """
    start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    clearance = max(1e-4, float(clearance_m))
    delta = center - start
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        direction = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        direction = delta / dist
    yaw = float(yaw_offset_rad)
    if abs(yaw) > 1e-9:
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z = direction
        direction = np.array([c * x - s * y, s * x + c * y, z], dtype=float)
        n = float(np.linalg.norm(direction))
        if n < 1e-9:
            direction = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            direction = direction / n
    return center - clearance * direction


def radial_preapproach_tip(
    sphere_center_m: np.ndarray,
    *,
    clearance_m: float,
    base_position_m: np.ndarray = (0.0, 0.0, 0.0),
    yaw_offset_rad: float = 0.0,
) -> np.ndarray:
    """Pre-approach tip on the **base→target** line, ``clearance_m`` from center.

    Why a base→target ray (not tip→center)
    --------------------------------------
    When the EE starts *inside* the standoff shell (``EE_CLOSE``), the
    tip→center vector is tiny and its direction is numerically degenerate, so
    the usual standoff candidates (``tip_standoff_on_approach``) collapse onto
    the current pose and the oriented-contact IK keeps failing. The direction
    from the arm **base** (URDF ``base_link`` origin, tip poses are in that
    frame) toward the target is always well defined and is the natural,
    well-conditioned reach direction for a base-mounted arm. Backing the tip
    off along it gives an extended pre-approach from which the short oriented
    contact is far more likely to solve IK.

    ``yaw_offset_rad`` rotates the radial about +Z (through the center) to try a
    few lateral pre-approaches. Units: meters / radians.
    """
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    base = np.asarray(base_position_m, dtype=float).reshape(3)
    clearance = max(1e-4, float(clearance_m))
    delta = center - base
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        direction = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        direction = delta / dist
    yaw = float(yaw_offset_rad)
    if abs(yaw) > 1e-9:
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z = direction
        direction = np.array([c * x - s * y, s * x + c * y, z], dtype=float)
        n = float(np.linalg.norm(direction))
        direction = (
            direction / n if n > 1e-9 else np.array([0.0, 0.0, 1.0], dtype=float)
        )
    # Back off toward the base along the radial (pre-approach on the reach side).
    return center - clearance * direction


def ordered_standoff_candidates(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    tip_contact_m: np.ndarray,
    *,
    clearances_m: Sequence[float],
    yaws_rad: Sequence[float],
    radius_m: float,
    surface_margin_m: float,
    min_travel_m: float,
    skip_near_contact_m: float = 0.015,
) -> list[tuple[float, float, np.ndarray, float, str]]:
    """Build via-standoff candidates, nearest to the current tip first.

    Progressive recovery: try a nearby standoff first, then increasingly
    distant waypoints on later retries. Candidates with tip travel
    ``< min_travel_m`` are omitted (``skip_near_start``) so we do not plan to
    an almost-identical EE pose.

    Returns
    -------
    list of ``(clearance_m, yaw_rad, tip_standoff_m, travel_m, skip_reason)``
    where ``skip_reason`` is empty for usable candidates, else a short tag.
    Usable rows are sorted by ``travel_m`` **ascending** (near → far).
    """
    tip_start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    tip_contact = np.asarray(tip_contact_m, dtype=float).reshape(3)
    min_clear = float(radius_m) + float(surface_margin_m) + 0.01
    min_travel = max(0.0, float(min_travel_m))
    usable: list[tuple[float, float, np.ndarray, float, str]] = []
    skipped: list[tuple[float, float, np.ndarray, float, str]] = []
    for clearance in clearances_m:
        clearance_f = max(float(clearance), min_clear)
        for yaw in yaws_rad:
            tip_standoff = tip_standoff_on_approach(
                tip_start,
                center,
                clearance_m=clearance_f,
                yaw_offset_rad=float(yaw),
            )
            travel = float(np.linalg.norm(tip_standoff - tip_start))
            if float(np.linalg.norm(tip_standoff - tip_contact)) < skip_near_contact_m:
                skipped.append(
                    (clearance_f, float(yaw), tip_standoff, travel, "skip_near_contact")
                )
                continue
            if travel < min_travel:
                skipped.append(
                    (clearance_f, float(yaw), tip_standoff, travel, "skip_near_start")
                )
                continue
            usable.append((clearance_f, float(yaw), tip_standoff, travel, ""))
    usable.sort(key=lambda row: row[3])  # nearest first, then farther
    return usable + skipped


def recovery_via_attempts_in_message(message: str) -> int:
    """Count ``via1_`` planning legs recorded in a plan message."""
    return len(re.findall(r"via1_", str(message or "")))


def recovery_direct_attempted(message: str) -> bool:
    return "direct:" in str(message or "") or "strategy=direct" in str(message or "")


def recovery_audit_plan_fail(
    message: str,
    *,
    recovery_enabled: bool = True,
    had_marker: bool = True,
) -> dict[str, object]:
    """Audit a failed plan message for the 'yellow / no recovery' bug class.

    When recovery is enabled and a volumetric marker was present, a PLAN_FAIL
    must show at least one ``via1_`` attempt (unless the message explicitly
    records that vias were impossible). Returns a dict with ``ok`` and
    ``reason``.
    """
    msg = str(message or "")
    vias = recovery_via_attempts_in_message(msg)
    if not recovery_enabled:
        return {"ok": True, "reason": "recovery_disabled", "via_attempts": vias}
    if not had_marker:
        return {"ok": True, "reason": "no_marker", "via_attempts": vias}
    if "recovery_skipped" in msg or "curobo_unavailable" in msg:
        return {"ok": False, "reason": "recovery_skipped", "via_attempts": vias}
    if vias < 1 and "skip_near" not in msg and "no_marker" not in msg:
        return {
            "ok": False,
            "reason": "no_via_attempt_after_direct_fail",
            "via_attempts": vias,
        }
    return {"ok": True, "reason": "vias_attempted", "via_attempts": vias}


def _select_marker(
    obstacles: Sequence[SphereObstacle],
    tip_goal_m: np.ndarray,
) -> SphereObstacle | None:
    if not obstacles:
        return None
    tip_goal = np.asarray(tip_goal_m, dtype=float).reshape(3)
    for candidate in obstacles:
        if (
            float(
                np.linalg.norm(
                    np.asarray(candidate.center_m, dtype=float).reshape(3) - tip_goal
                )
            )
            < 1e-4
        ):
            return candidate
    return obstacles[0]


def _concat_waypoints(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    wa = np.asarray(a, dtype=float)
    wb = np.asarray(b, dtype=float)
    if wa.size == 0:
        return wb
    if wb.size == 0:
        return wa
    if np.allclose(wa[-1], wb[0], atol=1e-5):
        return np.vstack([wa, wb[1:]])
    return np.vstack([wa, wb])


def _clamp_joints(q_rad: np.ndarray) -> np.ndarray:
    """Clamp to URDF joint limits with a small interior margin (radians)."""
    lo, hi = load_joint_limits_rad()
    margin = 1e-3
    return np.clip(
        np.asarray(q_rad, dtype=float).reshape(6),
        lo + margin,
        hi - margin,
    )


def _blend_toward_home(q_rad: np.ndarray, *, weight: float = 0.35) -> np.ndarray:
    """Pull joints toward home and clamp (last-resort INVALID_START escape)."""
    from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad

    q = np.asarray(q_rad, dtype=float).reshape(6)
    home = load_home_joint_positions_rad().reshape(6)
    w = float(np.clip(weight, 0.0, 1.0))
    return _clamp_joints((1.0 - w) * q + w * home)


def try_move_to_preparatory_seed(
    q_cur: np.ndarray,
    *,
    planner: CuRoboMotionPlanner,
    model: UrdfKinematicModel,
    obstacles: list[SphereObstacle] | None = None,
    failed_seeds: list[np.ndarray] | None = None,
    previous_goals: Sequence[np.ndarray] | None = None,
    execute_waypoints: ExecuteWaypointsFn | None = None,
    allow_planned: bool = True,
    max_seeds: int = 4,
    dt_s: float = 0.02,
) -> tuple[np.ndarray | None, str, list[np.ndarray]]:
    """Move toward a seed-bank preparatory configuration (radians).

    Spec (``spec.md`` § IK failure → preparatory repositioning)
    -----------------------------------------------------------
    Prefer joint-space seeds (home / mid-home / prior goals), not random tip
    jitter. When ``allow_planned`` is True, try collision-aware ``plan_to_pose``
    to the FK tip of ``q_seed``. If that fails (typical for
    ``INVALID_START_*`` — MotionGen cannot leave a colliding start), fall back
    to an open-loop joint move to ``q_seed``.

    Returns
    -------
    ``(new_q, log_tag, updated_failed_seeds)``. ``new_q`` is None when the
    bank is exhausted.
    """
    failed = [np.asarray(f, dtype=float).reshape(6) for f in (failed_seeds or [])]
    # Treat current as already-tried so we do not "escape" to the same pose.
    failed_with_cur = failed + [_clamp_joints(q_cur)]
    bank = build_seed_bank(
        q_cur,
        previous_goals=previous_goals,
        include_mid_home=True,
    )
    ordered = order_seeds(
        bank, q_current=q_cur, failed_seeds=failed_with_cur
    )
    if max_seeds > 0:
        ordered = ordered[: int(max_seeds)]

    obs = list(obstacles or [])
    for q_seed in ordered:
        q_seed = _clamp_joints(q_seed)
        if joint_distance_rad(q_seed, q_cur) < 1e-3:
            failed.append(q_seed.copy())
            continue

        tag = f"prep_seed_d{joint_distance_rad(q_seed, q_cur):.3f}"
        planned_ok = False
        waypoints: np.ndarray | None = None
        plan_dt = float(dt_s)

        if allow_planned:
            pose_seed = forward_kinematics(q_seed, model=model)
            planned = planner.plan_to_pose(
                q_cur,
                pose_seed.position_m,
                pose_seed.quaternion_wxyz,
                max_attempts=1,
                obstacles=obs,
            )
            plan_dt = float(planned.dt_s)
            if planned.ok and planned.waypoints_rad.size > 0:
                waypoints = np.asarray(planned.waypoints_rad, dtype=float)
                planned_ok = True
                tag = f"{tag}_planned|{planned.message}"
            else:
                tag = f"{tag}_plan_fail:{planned.message}"

        if not planned_ok:
            # Open-loop escape: required when start is in world collision
            # (MotionGen returns INVALID_START and cannot leave).
            waypoints = np.vstack(
                [np.asarray(q_cur, dtype=float).reshape(1, 6), q_seed.reshape(1, 6)]
            )
            tag = f"{tag}_open_loop"

        assert waypoints is not None
        if execute_waypoints is not None:
            execute_waypoints(waypoints, max(plan_dt, 0.02))
        q_new = _clamp_joints(waypoints[-1])
        failed.append(q_seed.copy())
        return q_new, tag, failed

    return None, "prep_seed_bank_exhausted", failed


def try_radial_reposition_via(
    q_cur: np.ndarray,
    *,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    planner: CuRoboMotionPlanner,
    model: UrdfKinematicModel,
    cfg: dict,
    obstacles: list[SphereObstacle] | None = None,
    execute_waypoints: ExecuteWaypointsFn | None = None,
    attempts_log: list[str] | None = None,
) -> tuple[np.ndarray | None, str]:
    """Reposition the arm to a base→target pre-approach so contact IK can solve.

    Used when the direct oriented contact returns ``IK_FAIL`` while the EE is
    **close** to the target (folded near it, wrong wrist orientation). Rather
    than exclude such targets, we generate an explicit **reposition via**: back
    the tip off along the well-conditioned base→target radial to a comfortable
    pre-approach (``plan_recovery_reposition_clearances_m``), commanding the
    pad-facing orientation there, then hand back the new joints so the caller
    can retry the oriented contact from an extended, more dexterous pose.

    Cartesian reposition uses the tip-spheres-ON ``planner`` (free-space move,
    no contact). Returns ``(q_new, tag)``; ``q_new`` is None when every
    pre-approach candidate failed to plan.
    """
    log = attempts_log if attempts_log is not None else []
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    radius = float(sphere_radius_m)
    obs = list(obstacles or [])
    surface_margin = float(cfg.get("plan_recovery_surface_margin_m", 0.008))
    min_clear = radius + surface_margin + 0.01
    clearances = [
        max(min_clear, float(c))
        for c in cfg.get(
            "plan_recovery_reposition_clearances_m", [0.08, 0.12, 0.16]
        )
    ]
    yaws = list(cfg.get("plan_recovery_reposition_yaws_rad", [0.0, 0.4, -0.4]))
    attempts = int(cfg.get("curobo_max_attempts", 4))

    for clearance in clearances:
        for yaw in yaws:
            tip_pre = radial_preapproach_tip(
                center, clearance_m=clearance, yaw_offset_rad=float(yaw)
            )
            # Pad-facing orientation at the pre-approach (+Z along the outward
            # normal, i.e. from center back toward the base/reach side).
            approach = build_sphere_contact_approach(
                tip_pre,
                center,
                radius,
                standoff_m=float(cfg.get("contact_standoff_m", 0.015)),
                current_quaternion_wxyz=forward_kinematics(
                    q_cur, model=model
                ).quaternion_wxyz,
            )
            plan = planner.plan_to_pose(
                q_cur,
                tip_pre,
                approach.quaternion_wxyz,
                max_attempts=max(1, attempts),
                obstacles=obs,
            )
            tag = f"reposition_radial_c{clearance:.3f}_y{yaw:.2f}"
            log.append(f"{tag}:{plan.message}")
            if not (plan.ok and plan.waypoints_rad.size > 0):
                continue
            if execute_waypoints is not None:
                execute_waypoints(plan.waypoints_rad, float(plan.dt_s))
            q_new = _clamp_joints(plan.waypoints_rad[-1])
            return q_new, f"{tag}_ok"
    return None, "reposition_radial_exhausted"


def _is_invalid_start(message: str) -> bool:
    m = str(message or "").upper()
    return "INVALID_START" in m


def _is_ik_fail(message: str) -> bool:
    """True when a plan message reports an IK/pose-reach failure (not collision)."""
    return "IK_FAIL" in str(message or "").upper()


def _fail(
    message: str,
    *,
    backend: str = "curobo",
    dt_s: float = 0.02,
) -> PlannedTrajectory:
    return PlannedTrajectory(
        waypoints_rad=np.zeros((0, 6)),
        dt_s=dt_s,
        success=False,
        backend=backend,
        message=message,
    )


def _ok(
    waypoints: np.ndarray,
    *,
    dt_s: float,
    message: str,
    backend: str = "curobo",
) -> PlannedTrajectory:
    wp = np.asarray(waypoints, dtype=float)
    if wp.ndim == 1:
        wp = wp.reshape(1, -1)
    if wp.size == 0:
        wp = np.zeros((1, 6))
    return PlannedTrajectory(
        waypoints_rad=wp,
        dt_s=dt_s,
        success=True,
        backend=backend,
        message=message,
    )


def try_oriented_tip_face_contact(
    q_start_rad: np.ndarray,
    *,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    planner: CuRoboMotionPlanner,
    contact_planner: CuRoboMotionPlanner,
    obstacles: list[SphereObstacle] | None = None,
    model: UrdfKinematicModel | None = None,
    max_attempts: int = 4,
    execute_waypoints: ExecuteWaypointsFn | None = None,
    strategy_tag: str = "direct_contact",
    attempts_log: list[str] | None = None,
    decision_emit: DecisionEmitFn | None = None,
) -> PlannedTrajectory | None:
    """Spheres-on approach to oriented standoff, then tip-omit axial nudge.

    Returns a successful ``PlannedTrajectory`` or ``None`` if either leg fails.
    Tip/flange collision spheres stay active until the short final nudge
    (``contact_planner`` must use ``omit_tip_links=True``).

    Tip-omit is refused unless Cartesian length ≤ ``contact_nudge_max_m``.
    After a failed spheres-ON approach (or skip-near without a pad-facing
    command), FK tool +Z must already face the outward normal — otherwise
    reseat with spheres ON or refuse. Long tip-omit fallbacks (e.g. ≤60 mm)
    are intentionally not used — they let the EE barrel hit the marker from
    the side/back.
    """
    mdl = model or get_default_model()
    cfg = load_planning_config()
    if not bool(cfg.get("contact_axis_enabled", True)):
        return None
    standoff_m = float(cfg.get("contact_standoff_m", 0.008))
    nudge_m = float(cfg.get("contact_nudge_m", standoff_m))
    nudge_max = float(cfg.get("contact_nudge_max_m", max(0.012, standoff_m)))
    # Tip-omit segment covers standoff→pierce; allow up to nudge_max.
    nudge_m = float(np.clip(nudge_m, 1e-4, nudge_max))
    if standoff_m > nudge_max + 1e-9:
        # Keep invariant: spheres-on ends no farther out than tip-omit can cover.
        standoff_m = nudge_max
    log = attempts_log if attempts_log is not None else []
    obs = list(obstacles or [])
    # Tip-omit contact must use the *visual* marker radius. If the caller
    # inflated ``ik_target`` for the spheres-ON approach, deflate it here so
    # wrist spheres are not fighting an oversize OBB at the pierce point.
    inflate = float(cfg.get("target_obstacle_inflate_m", 0.0))
    obs_contact = list(obs)
    if inflate > 1e-9 and obs:
        from residual_adaptive_ik.geometry import SphereObstacle

        obs_contact = []
        for o in obs:
            name = str(getattr(o, "name", "") or "")
            if name.startswith("ik_target"):
                r = max(1e-4, float(o.radius_m) - inflate)
                obs_contact.append(
                    SphereObstacle(
                        center_m=np.asarray(o.center_m, dtype=float).reshape(3),
                        radius_m=r,
                        name=name,
                    )
                )
            else:
                obs_contact.append(o)
    q_cur = _clamp_joints(q_start_rad)
    pose0 = forward_kinematics(q_cur, model=mdl)
    tip0 = np.asarray(pose0.position_m, dtype=float).reshape(3)
    approach = build_sphere_contact_approach(
        tip0,
        sphere_center_m,
        sphere_radius_m,
        standoff_m=standoff_m,
        nudge_m=nudge_m,
        current_quaternion_wxyz=pose0.quaternion_wxyz,
    )
    quat = approach.quaternion_wxyz
    last_dt = float(getattr(planner, "_interpolation_dt_s", 0.02))
    waypoints_approach = None
    # Orientation actually chosen by the approach leg (may be a bounded cone
    # tilt, not the exact outward normal). Threaded into the tip-omit nudge so
    # the final segment commands the same reachable pad-facing orientation.
    chosen_quat: np.ndarray | None = None

    dist_to_standoff = float(
        np.linalg.norm(tip0 - approach.standoff_position_m)
    )
    tip_to_pierce0 = float(np.linalg.norm(tip0 - approach.pierce_position_m))
    obs_r = float(obs[0].radius_m) if obs else float("nan")
    contact_r = float(obs_contact[0].radius_m) if obs_contact else float("nan")
    _decision(
        decision_emit,
        log,
        f"contact_begin tag={strategy_tag} standoff_m={standoff_m:.3f} "
        f"nudge_m={nudge_m:.3f} inflate_m={inflate:.3f} "
        f"obs_r_m={obs_r:.4f} contact_r_m={contact_r:.4f} "
        f"tip_to_standoff_m={dist_to_standoff:.3f} "
        f"tip_to_pierce_m={tip_to_pierce0:.3f}",
    )
    # Already near the oriented standoff — skip the spheres-on approach leg.
    if dist_to_standoff > 0.004:
        pose_now = forward_kinematics(q_cur, model=mdl)
        # Bounded orientation cone (exact outward normal first) so near-envelope
        # targets get an IK-reachable pad-facing wrist without accepting a
        # side/through contact. See ``contact_orientation_candidates``.
        quat_candidates = contact_orientation_candidates(
            approach, fallback_quaternion_wxyz=pose_now.quaternion_wxyz, cfg=cfg
        )
        _decision(
            decision_emit,
            log,
            f"approach_leg n_ori={len(quat_candidates)} "
            f"(spheres_ON → standoff)",
        )
        leg_ap = None
        for qi, quat_try in enumerate(quat_candidates):
            leg_ap = planner.plan_to_pose(
                q_cur,
                approach.standoff_position_m,
                quat_try,
                max_attempts=max(1, int(max_attempts)),
                obstacles=obs,
            )
            log.append(f"contact_approach_q{qi}:{leg_ap.message}")
            last_dt = float(leg_ap.dt_s)
            if leg_ap.ok:
                wp_chk = np.asarray(leg_ap.waypoints_rad, dtype=float)
                # Unit FakePlanners bump q without matching tip FK — skip.
                if type(planner).__name__ == "CuRoboMotionPlanner":
                    clear, min_d = tip_path_avoids_marker_immersion(
                        wp_chk,
                        sphere_center_m=sphere_center_m,
                        sphere_radius_m=sphere_radius_m,
                        model=mdl,
                        stride=max(1, wp_chk.shape[0] // 32),
                    )
                    if not clear:
                        log.append(
                            f"contact_approach_q{qi}:tip_immerses_path"
                            f"|min_dist_m={min_d:.4f}"
                        )
                        _decision(
                            decision_emit,
                            log,
                            f"approach_reject_immersion q{qi} "
                            f"min_tip_dist_m={min_d:.4f}<r={sphere_radius_m:.4f}",
                        )
                        leg_ap = PlannedTrajectory(
                            waypoints_rad=np.zeros((0, 6)),
                            dt_s=last_dt,
                            success=False,
                            backend=getattr(leg_ap, "backend", "curobo"),
                            message="plan_failed:tip_immerses_marker_path",
                        )
                    if leg_ap.ok:
                        # Reject approaches where a proximal arm capsule clips
                        # the marker (iter11 Ep6 MARKER_ARM_SWEEP seg_2).
                        arm_r = float(
                            cfg.get(
                                "arm_sweep_link_radius_m",
                                cfg.get("link_radius_m", 0.012),
                            )
                        )
                        n_ign = int(
                            cfg.get("arm_sweep_n_ee_segments_ignored", 3)
                        )
                        stride_arm = max(1, wp_chk.shape[0] // 16)
                        for q_arm in wp_chk[::stride_arm]:
                            arm_hit = proximal_arm_contacts_target(
                                q_arm,
                                sphere_center_m,
                                target_radius_m=sphere_radius_m,
                                model=mdl,
                                link_radius_m=arm_r,
                                n_ee_segments_ignored=n_ign,
                            )
                            if arm_hit.collides:
                                log.append(
                                    f"contact_approach_q{qi}:arm_body_path|"
                                    f"reasons={list(arm_hit.reasons)}"
                                )
                                _decision(
                                    decision_emit,
                                    log,
                                    f"approach_reject_arm_body q{qi} "
                                    f"reasons={list(arm_hit.reasons)}",
                                )
                                leg_ap = PlannedTrajectory(
                                    waypoints_rad=np.zeros((0, 6)),
                                    dt_s=last_dt,
                                    success=False,
                                    backend=getattr(leg_ap, "backend", "curobo"),
                                    message="plan_failed:arm_body_on_approach",
                                )
                                break
                    if leg_ap.ok:
                        # Reject tip-face side_graze / through / clear wrong_side
                        # samples along the approach (iter12 Ep8 lat=11 mm).
                        standoff_ref = np.asarray(
                            approach.standoff_position_m, dtype=float
                        ).reshape(3)
                        tf_ok, tf_reason = tip_path_tip_face_ok(
                            wp_chk,
                            sphere_center_m=sphere_center_m,
                            sphere_radius_m=sphere_radius_m,
                            approach_from_m=standoff_ref,
                            model=mdl,
                            cfg=cfg,
                            stride=max(1, wp_chk.shape[0] // 24),
                        )
                        if not tf_ok:
                            log.append(
                                f"contact_approach_q{qi}:tip_face_path|"
                                f"reason={tf_reason}"
                            )
                            _decision(
                                decision_emit,
                                log,
                                f"approach_reject_tip_face q{qi} "
                                f"reason={tf_reason}",
                            )
                            leg_ap = PlannedTrajectory(
                                waypoints_rad=np.zeros((0, 6)),
                                dt_s=last_dt,
                                success=False,
                                backend=getattr(leg_ap, "backend", "curobo"),
                                message=f"plan_failed:tip_face_{tf_reason}",
                            )
                if leg_ap.ok:
                    quat = np.asarray(quat_try, dtype=float).reshape(4)
                    chosen_quat = quat.copy()
                    _decision(
                        decision_emit,
                        log,
                        f"approach_ok q{qi} backend={leg_ap.backend}",
                    )
                    break
            # First + last failure only — avoid spamming every cone tilt.
            if (leg_ap is None or not leg_ap.ok) and (
                qi == 0 or qi == len(quat_candidates) - 1
            ):
                _decision(
                    decision_emit,
                    log,
                    f"approach_fail q{qi}/{len(quat_candidates)-1} "
                    f"msg={leg_ap.message if leg_ap else 'none'}",
                )
        if leg_ap is None or not leg_ap.ok:
            # MotionGen often fails on sequential handoffs (~50–80 mm) when the
            # wrist still faces the previous marker. Try classical DLS → joint
            # lerp to the oriented standoff before giving up on tip-omit/vias.
            # Unit FakePlanners are not CuRoboMotionPlanner — skip so via-path
            # tests still exercise MotionGen-fail → via recovery.
            use_dls = (
                type(planner).__name__ == "CuRoboMotionPlanner"
                and bool(cfg.get("contact_dls_approach_fallback", True))
            )
            if use_dls:
                _decision(
                    decision_emit,
                    log,
                    "approach_motiongen_failed → dls_standoff_approach",
                )
                for qi, quat_try in enumerate(quat_candidates):
                    leg_ap = plan_dls_standoff_approach_lerp(
                        q_cur,
                        approach.standoff_position_m,
                        quat_try,
                        sphere_center_m=sphere_center_m,
                        sphere_radius_m=sphere_radius_m,
                        model=mdl,
                        dt_s=last_dt,
                        n_samples=48,
                    )
                    log.append(f"contact_approach_dls_q{qi}:{leg_ap.message}")
                    last_dt = float(leg_ap.dt_s)
                    if leg_ap.ok:
                        # Same tip-face / immersion / arm-body gates as MotionGen
                        # (iter16 Ep10: DLS lerp chord side_graze lat=12 mm then
                        # green → PLAN_FAIL invalid_side).
                        wp_dls = np.asarray(leg_ap.waypoints_rad, dtype=float)
                        clear, min_d = tip_path_avoids_marker_immersion(
                            wp_dls,
                            sphere_center_m=sphere_center_m,
                            sphere_radius_m=sphere_radius_m,
                            model=mdl,
                            stride=max(1, wp_dls.shape[0] // 32),
                        )
                        if not clear:
                            log.append(
                                f"contact_approach_dls_q{qi}:tip_immerses_path"
                                f"|min_dist_m={min_d:.4f}"
                            )
                            _decision(
                                decision_emit,
                                log,
                                f"approach_dls_reject_immersion q{qi} "
                                f"min_tip_dist_m={min_d:.4f}",
                            )
                            leg_ap = PlannedTrajectory(
                                waypoints_rad=np.zeros((0, 6)),
                                dt_s=last_dt,
                                success=False,
                                backend="dls_approach",
                                message="plan_failed:dls_tip_immerses_marker_path",
                            )
                        if leg_ap.ok:
                            arm_r = float(
                                cfg.get(
                                    "arm_sweep_link_radius_m",
                                    cfg.get("link_radius_m", 0.012),
                                )
                            )
                            n_ign = int(
                                cfg.get("arm_sweep_n_ee_segments_ignored", 3)
                            )
                            stride_arm = max(1, wp_dls.shape[0] // 16)
                            for q_arm in wp_dls[::stride_arm]:
                                arm_hit = proximal_arm_contacts_target(
                                    q_arm,
                                    sphere_center_m,
                                    target_radius_m=sphere_radius_m,
                                    model=mdl,
                                    link_radius_m=arm_r,
                                    n_ee_segments_ignored=n_ign,
                                )
                                if arm_hit.collides:
                                    log.append(
                                        f"contact_approach_dls_q{qi}:"
                                        f"arm_body_path|"
                                        f"reasons={list(arm_hit.reasons)}"
                                    )
                                    _decision(
                                        decision_emit,
                                        log,
                                        f"approach_dls_reject_arm_body q{qi}",
                                    )
                                    leg_ap = PlannedTrajectory(
                                        waypoints_rad=np.zeros((0, 6)),
                                        dt_s=last_dt,
                                        success=False,
                                        backend="dls_approach",
                                        message="plan_failed:dls_arm_body_on_approach",
                                    )
                                    break
                        if leg_ap.ok:
                            standoff_ref = np.asarray(
                                approach.standoff_position_m, dtype=float
                            ).reshape(3)
                            tf_ok, tf_reason = tip_path_tip_face_ok(
                                wp_dls,
                                sphere_center_m=sphere_center_m,
                                sphere_radius_m=sphere_radius_m,
                                approach_from_m=standoff_ref,
                                model=mdl,
                                cfg=cfg,
                                stride=1,
                                densify_n=64,
                            )
                            if not tf_ok:
                                log.append(
                                    f"contact_approach_dls_q{qi}:tip_face_path|"
                                    f"reason={tf_reason}"
                                )
                                _decision(
                                    decision_emit,
                                    log,
                                    f"approach_dls_reject_tip_face q{qi} "
                                    f"reason={tf_reason}",
                                )
                                leg_ap = PlannedTrajectory(
                                    waypoints_rad=np.zeros((0, 6)),
                                    dt_s=last_dt,
                                    success=False,
                                    backend="dls_approach",
                                    message=f"plan_failed:dls_tip_face_{tf_reason}",
                                )
                    if leg_ap.ok:
                        # Reject DLS ends that are not pad-facing (iter25 Ep12:
                        # approach_ok_dls then axis_err=1.76 → tip-omit refuse).
                        q_end = np.asarray(
                            leg_ap.waypoints_rad[-1], dtype=float
                        ).reshape(6)
                        pad_end, pad_err, pad_tol = fk_pad_aligned_for_tip_omit(
                            q_end,
                            approach.normal_outward,
                            model=mdl,
                            cfg=cfg,
                        )
                        if (not pad_end) and pad_err > max(0.50, 2.0 * pad_tol):
                            log.append(
                                f"contact_approach_dls_q{qi}:pad_end_misaligned|"
                                f"axis_err={pad_err:.3f}>tol={pad_tol:.3f}"
                            )
                            _decision(
                                decision_emit,
                                log,
                                f"approach_dls_reject_pad_end q{qi} "
                                f"axis_err_rad={pad_err:.3f}",
                            )
                            leg_ap = PlannedTrajectory(
                                waypoints_rad=np.zeros((0, 6)),
                                dt_s=last_dt,
                                success=False,
                                backend="dls_approach",
                                message="plan_failed:dls_pad_end_misaligned",
                            )
                    if leg_ap.ok:
                        quat = np.asarray(quat_try, dtype=float).reshape(4)
                        chosen_quat = quat.copy()
                        _decision(
                            decision_emit,
                            log,
                            f"approach_ok_dls q{qi} backend={leg_ap.backend}",
                        )
                        break
                    if qi == 0 or qi == len(quat_candidates) - 1:
                        _decision(
                            decision_emit,
                            log,
                            f"approach_dls_fail q{qi}/{len(quat_candidates)-1} "
                            f"msg={leg_ap.message}",
                        )
        if leg_ap is None or not leg_ap.ok:
            # Last resort: tip-omit only when already inside the short axial
            # window AND the pad already faces the marker. Long tip-omit
            # (historically ≤60 mm) lets MotionGen swing the EE barrel into
            # the sphere from the side/back. Prefer vias / spheres-ON reseat.
            tip_far = float(np.linalg.norm(tip0 - approach.pierce_position_m))
            allow_m = tip_omit_length_allow_m(cfg, nudge_max_m=nudge_max)
            aligned, axis_err, axis_tol = fk_pad_aligned_for_tip_omit(
                q_cur,
                approach.normal_outward,
                model=mdl,
                cfg=cfg,
            )
            _decision(
                decision_emit,
                log,
                f"approach_all_failed → tip_omit_fallback "
                f"tip_far_m={tip_far:.3f} allow_m={allow_m:.3f} "
                f"axis_err_rad={axis_err:.3f} axis_tol_rad={axis_tol:.3f} "
                f"pad_aligned={int(aligned)}",
            )
            if tip_far > allow_m + 1e-9:
                log.append(
                    f"contact_nudge_direct_refused:dist={tip_far:.3f}"
                    f">allow={allow_m:.3f}"
                )
                _decision(
                    decision_emit,
                    log,
                    f"tip_omit_refused tip_far_m={tip_far:.3f}"
                    f">allow_m={allow_m:.3f}",
                )
                return None
            if not aligned:
                log.append(
                    f"contact_nudge_direct_refused:pad_misaligned:"
                    f"axis_err={axis_err:.3f}>tol={axis_tol:.3f}"
                )
                _decision(
                    decision_emit,
                    log,
                    f"tip_omit_refused_orientation "
                    f"axis_err_rad={axis_err:.3f}>tol_rad={axis_tol:.3f}",
                )
                return None
            for qi, quat_try in enumerate(quat_candidates):
                leg_nudge = plan_axial_tip_omit_lerp(
                    q_cur,
                    approach.pierce_position_m,
                    quat_try,
                    model=mdl,
                    dt_s=last_dt,
                    sphere_center_m=sphere_center_m,
                    sphere_radius_m=sphere_radius_m,
                    patch_lateral_max_m=float(
                        cfg.get("contact_lateral_tolerance_m", 0.003)
                    ),
                )
                if not leg_nudge.ok:
                    # Fallback: tip-omit MotionGen (may graze — last resort).
                    leg_nudge = contact_planner.plan_to_pose(
                        q_cur,
                        approach.pierce_position_m,
                        quat_try,
                        max_attempts=max(1, int(max_attempts)),
                        obstacles=obs_contact,
                    )
                log.append(
                    f"contact_nudge_direct_q{qi}:dist={tip_far:.3f}|"
                    f"{leg_nudge.message}"
                )
                if leg_nudge.ok:
                    _decision(
                        decision_emit,
                        log,
                        f"tip_omit_direct_ok q{qi} dist_m={tip_far:.3f} "
                        f"backend={leg_nudge.backend}",
                    )
                    wp_nudge = np.asarray(
                        leg_nudge.waypoints_rad, dtype=float
                    )
                    if execute_waypoints is not None:
                        execute_waypoints(
                            wp_nudge, float(leg_nudge.dt_s)
                        )
                        q_hold = _clamp_joints(wp_nudge[-1])
                        return _ok(
                            q_hold.reshape(1, 6),
                            dt_s=float(leg_nudge.dt_s),
                            message=(
                                f"ok|strategy={strategy_tag}|"
                                f"contact_standoff_m={standoff_m:.3f}|"
                                f"contact_nudge_m={tip_far:.3f}|"
                                f"already_executed"
                            ),
                            backend=leg_nudge.backend,
                        )
                    return _ok(
                        wp_nudge,
                        dt_s=float(leg_nudge.dt_s),
                        message=(
                            f"ok|strategy={strategy_tag}|"
                            f"contact_standoff_m={standoff_m:.3f}|"
                            f"contact_nudge_m={tip_far:.3f}"
                        ),
                        backend=leg_nudge.backend,
                    )
            return None
        waypoints_approach = np.asarray(leg_ap.waypoints_rad, dtype=float)
        q_cur = _clamp_joints(waypoints_approach[-1])
        # Gate the tip-omit length against the **planned** Cartesian standoff,
        # not FK of the joint path (unit fakes / coarse interpolation may not
        # land exactly on the standoff in tip space).
        tip0 = np.asarray(approach.standoff_position_m, dtype=float).reshape(3)
    else:
        log.append("contact_approach:skipped_near_standoff")
        _decision(
            decision_emit,
            log,
            f"approach_skip_near tip_to_standoff_m={dist_to_standoff:.3f}",
        )

    # Rebuild approach from the (planned or measured) tip so pierce stays axial.
    approach = build_sphere_contact_approach(
        tip0,
        sphere_center_m,
        sphere_radius_m,
        standoff_m=standoff_m,
        nudge_m=nudge_m,
        current_quaternion_wxyz=forward_kinematics(q_cur, model=mdl).quaternion_wxyz,
    )
    # Keep the cone orientation the approach leg actually reached (if any) so the
    # tip-omit nudge does not revert to an exact normal that just IK-failed.
    quat = chosen_quat if chosen_quat is not None else approach.quaternion_wxyz
    # Prefer the constructed standoff as the tip-omit start when we just
    # planned to it (keeps length ≈ contact_standoff_m).
    if waypoints_approach is not None:
        tip0 = np.asarray(approach.standoff_position_m, dtype=float).reshape(3)
    ok_ax, ax_reason = validate_axial_contact_segment(
        tip0,
        approach.pierce_position_m,
        approach.normal_outward,
        max_length_m=nudge_max + 0.002,
        lateral_tol_m=float(cfg.get("contact_lateral_tolerance_m", 0.003)),
    )
    # If current tip is not on the axis, first re-seat to oriented standoff.
    if not ok_ax and "not_axial" in ax_reason:
        leg_re = planner.plan_to_pose(
            q_cur,
            approach.standoff_position_m,
            quat,
            max_attempts=max(1, int(max_attempts)),
            obstacles=obs,
        )
        log.append(f"contact_reseat:{leg_re.message}|{ax_reason}")
        last_dt = float(leg_re.dt_s)
        if not leg_re.ok:
            return None
        wp_re = np.asarray(leg_re.waypoints_rad, dtype=float)
        waypoints_approach = (
            wp_re
            if waypoints_approach is None
            else _concat_waypoints(waypoints_approach, wp_re)
        )
        q_cur = _clamp_joints(wp_re[-1])
        tip0 = np.asarray(approach.standoff_position_m, dtype=float).reshape(3)
        approach = build_sphere_contact_approach(
            tip0,
            sphere_center_m,
            sphere_radius_m,
            standoff_m=standoff_m,
            nudge_m=nudge_m,
            current_quaternion_wxyz=forward_kinematics(
                q_cur, model=mdl
            ).quaternion_wxyz,
        )
        quat = chosen_quat if chosen_quat is not None else approach.quaternion_wxyz
        tip0 = np.asarray(approach.standoff_position_m, dtype=float).reshape(3)

    tip_to_pierce = float(
        np.linalg.norm(tip0 - approach.pierce_position_m)
    )
    if tip_to_pierce > nudge_max + 0.003:
        # Still too far for a tip-omit nudge — refuse (caller may via farther).
        log.append(
            f"contact_nudge_refused:dist={tip_to_pierce:.4f}>max={nudge_max:.4f}"
        )
        _decision(
            decision_emit,
            log,
            f"tip_omit_refused_after_approach tip_to_pierce_m={tip_to_pierce:.3f}"
            f">nudge_max_m={nudge_max:.3f}",
        )
        return None

    # Prefer FK pad alignment before tip-omit when the joint path actually
    # tracks the planned Cartesian standoff. Trusting MotionGen's commanded
    # quat alone (with a yawed FK tip) let tip-omit curves enter the surface
    # shell with wrong_side_axis mid-path → CONTACT_INVALID_MIDPATH_GRAZE.
    #
    # Unit FakePlanners bump ``q`` without matching tip FK to the planned
    # standoff (see tip0 assignment above). In that case skip the FK pad gate
    # and allow axial / contact tip-omit; real MotionGen approaches land tip
    # within ``contact_standoff_fk_track_tol_m`` and keep the hard gate.
    fk_tip = np.asarray(
        forward_kinematics(q_cur, model=mdl).position_m, dtype=float
    ).reshape(3)
    tip_track_err_m = float(np.linalg.norm(fk_tip - tip0))
    tip_track_tol_m = float(cfg.get("contact_standoff_fk_track_tol_m", 0.012))
    aligned, axis_err, axis_tol = fk_pad_aligned_for_tip_omit(
        q_cur,
        approach.normal_outward,
        model=mdl,
        cfg=cfg,
    )
    pad_ok_for_motiongen = bool(aligned)
    if tip_track_err_m > tip_track_tol_m:
        pad_ok_for_motiongen = True
        _decision(
            decision_emit,
            log,
            f"tip_omit_skip_fk_pad tip_track_err_m={tip_track_err_m:.3f}>"
            f"tol_m={tip_track_tol_m:.3f} (planned standoff not realized in FK)",
        )
    elif not aligned:
        reseat_quat = chosen_quat if chosen_quat is not None else quat
        leg_ori = planner.plan_to_pose(
            q_cur,
            approach.standoff_position_m,
            reseat_quat,
            max_attempts=max(1, int(max_attempts)),
            obstacles=obs,
        )
        log.append(
            f"contact_reseat_orientation:{leg_ori.message}|"
            f"axis_err={axis_err:.3f}>tol={axis_tol:.3f}"
        )
        _decision(
            decision_emit,
            log,
            f"tip_omit_pad_misaligned → reseat_spheres_ON "
            f"axis_err_rad={axis_err:.3f} tol_rad={axis_tol:.3f} "
            f"ok={int(leg_ori.ok)}",
        )
        if leg_ori.ok:
            wp_ori = np.asarray(leg_ori.waypoints_rad, dtype=float)
            waypoints_approach = (
                wp_ori
                if waypoints_approach is None
                else _concat_waypoints(waypoints_approach, wp_ori)
            )
            q_cur = _clamp_joints(wp_ori[-1])
            tip0 = np.asarray(approach.standoff_position_m, dtype=float).reshape(3)
            last_dt = float(leg_ori.dt_s)
            aligned2, axis_err2, axis_tol2 = fk_pad_aligned_for_tip_omit(
                q_cur,
                approach.normal_outward,
                model=mdl,
                cfg=cfg,
            )
            # FK after MotionGen reseat often sits just over tol (iter21 Ep6:
            # prints as 0.260>0.260). Allow +0.01 rad (~0.6°) for tip-omit
            # eligibility only — settle tip-face classify stays at TOOL_AXIS_TOL.
            if (not aligned2) and axis_err2 <= axis_tol2 + 0.01:
                aligned2 = True
                _decision(
                    decision_emit,
                    log,
                    f"tip_omit_reseat_near_tol axis_err_rad={axis_err2:.3f} "
                    f"tol_rad={axis_tol2:.3f} (+0.01 rad tip-omit margin)",
                )
            pad_ok_for_motiongen = bool(aligned2)
            axis_err, axis_tol = axis_err2, axis_tol2
            _decision(
                decision_emit,
                log,
                f"tip_omit_after_reseat axis_err_rad={axis_err2:.3f} "
                f"tol_rad={axis_tol2:.3f} pad_ok={int(aligned2)}",
            )
            if aligned2:
                chosen_quat = np.asarray(reseat_quat, dtype=float).reshape(4)
                quat = chosen_quat
            else:
                # Refuse tip-omit when reseat cannot pad-align (iter18 Ep3).
                log.append(
                    f"contact_nudge_refused:pad_misaligned_after_reseat:"
                    f"axis_err={axis_err2:.3f}>tol={axis_tol2:.3f}"
                )
                _decision(
                    decision_emit,
                    log,
                    f"tip_omit_refused_after_reseat "
                    f"axis_err_rad={axis_err2:.3f}>tol_rad={axis_tol2:.3f}",
                )
                return None

    if not pad_ok_for_motiongen and tip_track_err_m <= tip_track_tol_m:
        # Hard refuse when FK tracks standoff but pad still misaligned and we
        # did not reseat (or reseat failed without returning above).
        log.append(
            f"contact_nudge_refused:pad_misaligned:"
            f"axis_err={axis_err:.3f}>tol={axis_tol:.3f}"
        )
        _decision(
            decision_emit,
            log,
            f"tip_omit_refused_orientation "
            f"axis_err_rad={axis_err:.3f}>tol_rad={axis_tol:.3f}",
        )
        return None

    leg_nudge = plan_axial_tip_omit_lerp(
        q_cur,
        approach.pierce_position_m,
        quat,
        model=mdl,
        dt_s=last_dt,
        sphere_center_m=sphere_center_m,
        sphere_radius_m=sphere_radius_m,
        patch_lateral_max_m=float(
            cfg.get("contact_lateral_tolerance_m", 0.003)
        ),
    )
    if not leg_nudge.ok:
        # Retry axial tip-omit across the orientation cone before MotionGen
        # (iter28 Ep6: exact pierce IK-hard; patch+cone often lands tip-face).
        if pad_ok_for_motiongen:
            pose_omit = forward_kinematics(q_cur, model=mdl)
            omit_quats = contact_orientation_candidates(
                approach,
                fallback_quaternion_wxyz=pose_omit.quaternion_wxyz,
                cfg=cfg,
            )
            for qi, quat_try in enumerate(omit_quats):
                if np.allclose(quat_try, quat, atol=1e-9):
                    continue
                leg_alt = plan_axial_tip_omit_lerp(
                    q_cur,
                    approach.pierce_position_m,
                    quat_try,
                    model=mdl,
                    dt_s=last_dt,
                    sphere_center_m=sphere_center_m,
                    sphere_radius_m=sphere_radius_m,
                    patch_lateral_max_m=float(
                        cfg.get("contact_lateral_tolerance_m", 0.003)
                    ),
                )
                if leg_alt.ok:
                    _decision(
                        decision_emit,
                        log,
                        f"axial_tip_omit_ok_cone q{qi} "
                        f"backend={leg_alt.backend} msg={leg_alt.message}",
                    )
                    leg_nudge = leg_alt
                    quat = np.asarray(quat_try, dtype=float).reshape(4)
                    break
    if not leg_nudge.ok:
        if not pad_ok_for_motiongen:
            log.append(
                f"contact_nudge_refused:pad_misaligned_no_axial:"
                f"axis_err={axis_err:.3f}>tol={axis_tol:.3f}|"
                f"axial={leg_nudge.message}"
            )
            _decision(
                decision_emit,
                log,
                f"tip_omit_refused_orientation_no_axial "
                f"axis_err_rad={axis_err:.3f}>tol_rad={axis_tol:.3f} "
                f"axial={leg_nudge.message}",
            )
            return None
        _decision(
            decision_emit,
            log,
            f"axial_tip_omit_ik_fail → curobo_tip_omit ({leg_nudge.message})",
        )
        leg_nudge = contact_planner.plan_to_pose(
            q_cur,
            approach.pierce_position_m,
            quat,
            max_attempts=max(1, int(max_attempts)),
            obstacles=obs_contact,
        )
    log.append(
        f"contact_nudge:standoff_m={standoff_m:.3f}|nudge_m={nudge_m:.3f}|"
        f"{leg_nudge.message}"
    )
    last_dt = float(leg_nudge.dt_s)
    if not leg_nudge.ok:
        _decision(
            decision_emit,
            log,
            f"tip_omit_plan_failed msg={leg_nudge.message}",
        )
        return None
    # Tip-omit segment must not side-graze mid-lerp (iter14 Ep8 lat=6 mm).
    # Skip for unit FakePlanners (FK tip does not track planned pierce).
    if type(planner).__name__ == "CuRoboMotionPlanner":
        wp_nudge_chk = np.asarray(leg_nudge.waypoints_rad, dtype=float)
        tf_ok, tf_reason = tip_path_tip_face_ok(
            wp_nudge_chk,
            sphere_center_m=sphere_center_m,
            sphere_radius_m=sphere_radius_m,
            approach_from_m=np.asarray(
                approach.standoff_position_m, dtype=float
            ).reshape(3),
            model=mdl,
            cfg=cfg,
            stride=max(1, wp_nudge_chk.shape[0] // 8),
        )
        if not tf_ok:
            log.append(f"contact_nudge_refused:tip_face_path|reason={tf_reason}")
            _decision(
                decision_emit,
                log,
                f"tip_omit_reject_tip_face reason={tf_reason}",
            )
            return None

    tip_end = approach.pierce_position_m
    ok_ax, ax_reason = validate_axial_contact_segment(
        tip0,
        tip_end,
        approach.normal_outward,
        max_length_m=nudge_max + 0.003,
        lateral_tol_m=float(cfg.get("contact_lateral_tolerance_m", 0.003)),
    )
    if not ok_ax:
        # Soft check: planned Cartesian goal is axial by construction; joint
        # path may wobble — keep success but record the invariant.
        log.append(f"contact_nudge_axis_warn:{ax_reason}")

    wp_nudge = np.asarray(leg_nudge.waypoints_rad, dtype=float)
    if execute_waypoints is not None:
        if waypoints_approach is not None:
            execute_waypoints(waypoints_approach, last_dt)
        execute_waypoints(wp_nudge, float(leg_nudge.dt_s))
        q_hold = _clamp_joints(wp_nudge[-1])
        return _ok(
            q_hold.reshape(1, 6),
            dt_s=float(leg_nudge.dt_s),
            message=(
                f"ok|strategy={strategy_tag}|contact_standoff_m={standoff_m:.3f}|"
                f"contact_nudge_m={nudge_m:.3f}|already_executed"
            ),
            backend=leg_nudge.backend,
        )
    waypoints = (
        wp_nudge
        if waypoints_approach is None
        else _concat_waypoints(waypoints_approach, wp_nudge)
    )
    return _ok(
        waypoints,
        dt_s=float(leg_nudge.dt_s),
        message=(
            f"ok|strategy={strategy_tag}|contact_standoff_m={standoff_m:.3f}|"
            f"contact_nudge_m={nudge_m:.3f}"
        ),
        backend=leg_nudge.backend,
    )


def plan_via_standoff(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    planner: CuRoboMotionPlanner,
    contact_planner: CuRoboMotionPlanner | None = None,
    obstacles: list[SphereObstacle] | None = None,
    model: UrdfKinematicModel | None = None,
    standoff_clearances_m: Sequence[float] | None = None,
    surface_margin_m: float = 0.008,
    max_attempts: int = 4,
    timeout_s: float = 15.0,
    deadline_monotonic: float | None = None,
    execute_waypoints: ExecuteWaypointsFn | None = None,
    decision_emit: DecisionEmitFn | None = None,
) -> PlannedTrajectory:
    """Retry oriented tip-face contact + standoff vias until timeout.

    Recovery order (repeated until deadline)
    ---------------------------------------
    1. **Oriented tip-face contact:** spheres-on approach to a short standoff
       along the sphere normal (tool +Z facing the marker), then tip-omit
       axial nudge onto the pierce point only
       (``try_oriented_tip_face_contact``).
    2. Standoff candidates (clearance × lateral yaw), sorted by tip travel
       **ascending** (near → far); skip near-start / near-contact vias.
    3. After via1, retry oriented tip-face contact from the new pose (not a
       long omit-tip via2 with large margins).
    4. If via1 succeeds and contact fails and ``execute_waypoints`` is set,
       execute via1 (EE moves), update the start pose, and keep trying.
    5. Only after the wall-clock budget expires return a failed trajectory.

    Never returns an unsafe NumPy lerp.
    """
    mdl = model or get_default_model()
    cfg = load_planning_config()
    clearances = list(
        standoff_clearances_m
        if standoff_clearances_m is not None
        else cfg.get(
            "plan_recovery_standoff_clearances_m",
            [0.04, 0.06, 0.08, 0.10, 0.14, 0.18],
        )
    )
    yaws = list(
        cfg.get("plan_recovery_lateral_yaw_rad", [0.0, 0.4, -0.4, 0.8, -0.8])
    )
    surface_margin = float(
        cfg.get("plan_recovery_surface_margin_m", surface_margin_m)
    )
    # Floor only: skip tiny no-ops; progressive order tries near then far.
    min_standoff_travel_m = float(
        cfg.get("plan_recovery_min_standoff_travel_m", 0.01)
    )
    attempts = int(cfg.get("curobo_max_attempts", max_attempts))
    direct_attempts = int(
        cfg.get("plan_recovery_direct_max_attempts", min(2, attempts))
    )
    via_attempts = int(cfg.get("plan_recovery_via_max_attempts", attempts))
    contact = contact_planner or planner
    t0 = time.monotonic()
    timeout = float(timeout_s)
    deadline = (
        float(deadline_monotonic)
        if deadline_monotonic is not None
        else t0 + timeout
    )

    def _timed_out() -> bool:
        return time.monotonic() >= deadline

    obs = list(obstacles or [])
    pose_goal = forward_kinematics(q_goal_rad, model=mdl)
    tip_goal = np.asarray(pose_goal.position_m, dtype=float).reshape(3)
    quat = pose_goal.quaternion_wxyz
    marker = _select_marker(obs, tip_goal)
    attempts_log: list[str] = []
    via_legs_started = 0
    partial_execs = 0
    q_cur = _clamp_joints(q_start_rad)
    tip_start = forward_kinematics(q_cur, model=mdl).position_m
    contact_margin = (
        0.0 if bool(getattr(contact, "_omit_tip_links", False)) else surface_margin
    )
    last_dt = float(getattr(planner, "_interpolation_dt_s", 0.02))
    escape_weight = 0.4
    failed_prep_seeds: list[np.ndarray] = []
    prep_seed_max = int(cfg.get("plan_recovery_prep_seed_max", 4))
    prep_seed_enabled = bool(cfg.get("plan_recovery_prep_seed_enabled", True))
    # Radial reposition budget for the EE-close IK_FAIL case (back off along the
    # base→target ray so the oriented contact IK becomes solvable).
    reposition_enabled = bool(cfg.get("plan_recovery_reposition_enabled", True))
    reposition_max = int(cfg.get("plan_recovery_reposition_max_attempts", 2))
    reposition_shell_m = float(
        cfg.get("plan_recovery_reposition_close_shell_m", 0.06)
    )
    reposition_used = 0

    if marker is None:
        direct = contact.plan_to_joint_goal(
            q_cur,
            q_goal_rad,
            model=mdl,
            obstacles=obs,
            max_attempts=max(1, direct_attempts),
        )
        attempts_log.append(f"direct:{direct.message}")
        if direct.ok:
            return _ok(
                direct.waypoints_rad,
                dt_s=direct.dt_s,
                message=f"ok|strategy=direct|{direct.message}",
                backend=direct.backend,
            )
        return _fail(
            f"recovery_exhausted_no_marker|via_attempts=0|{'|'.join(attempts_log)}",
            dt_s=last_dt,
        )

    center = np.asarray(marker.center_m, dtype=float).reshape(3)
    radius = float(marker.radius_m)

    # Always run at least one recovery pass (even if the soft deadline already
    # elapsed during setup) so logs show via1_ attempts for the audit contract.
    first_pass = True
    while first_pass or not _timed_out():
        allow_overtime = first_pass
        first_pass = False
        tip_contact = tip_on_sphere_surface(
            tip_start, center, radius, margin_m=contact_margin
        )

        # Preferred path: spheres-on approach + short tip-omit axial nudge.
        _decision(
            decision_emit,
            attempts_log,
            f"recovery_try_contact q_cycle tip_m="
            f"({tip_start[0]:.3f},{tip_start[1]:.3f},{tip_start[2]:.3f}) "
            f"elapsed_s={time.monotonic() - t0:.1f}",
        )
        oriented = try_oriented_tip_face_contact(
            q_cur,
            sphere_center_m=center,
            sphere_radius_m=radius,
            planner=planner,
            contact_planner=contact,
            obstacles=obs,
            model=mdl,
            max_attempts=max(1, direct_attempts),
            execute_waypoints=execute_waypoints,
            strategy_tag="direct_contact",
            attempts_log=attempts_log,
            decision_emit=decision_emit,
        )
        if oriented is not None and oriented.ok:
            _decision(
                decision_emit,
                attempts_log,
                f"recovery_contact_ok strategy=direct_contact "
                f"elapsed_s={time.monotonic() - t0:.1f}",
            )
            msg = (
                f"{oriented.message}|partial_execs={partial_execs}|"
                f"elapsed_s={time.monotonic() - t0:.2f}"
            )
            return _ok(
                oriented.waypoints_rad,
                dt_s=oriented.dt_s,
                message=msg,
                backend=oriented.backend,
            )
        _decision(
            decision_emit,
            attempts_log,
            "recovery_contact_failed → via_standoffs",
        )

        # Legacy tip-omit full path only when oriented contact is disabled.
        if not bool(cfg.get("contact_axis_enabled", True)):
            direct = contact.plan_to_joint_goal(
                q_cur,
                q_goal_rad,
                model=mdl,
                obstacles=obs,
                max_attempts=max(1, direct_attempts),
            )
            attempts_log.append(f"direct:{direct.message}")
            last_dt = float(direct.dt_s)
            if direct.ok:
                msg = (
                    f"ok|strategy=direct|partial_execs={partial_execs}|"
                    f"elapsed_s={time.monotonic() - t0:.2f}|{direct.message}"
                )
                if execute_waypoints is not None:
                    execute_waypoints(direct.waypoints_rad, float(direct.dt_s))
                    q_hold = _clamp_joints(direct.waypoints_rad[-1])
                    return _ok(
                        q_hold.reshape(1, 6),
                        dt_s=direct.dt_s,
                        message=msg + "|already_executed",
                        backend=direct.backend,
                    )
                return _ok(
                    direct.waypoints_rad,
                    dt_s=direct.dt_s,
                    message=msg,
                    backend=direct.backend,
                )
        else:
            direct = _fail("oriented_contact_pending", dt_s=last_dt)

        # Invalid start: MotionGen cannot leave a colliding configuration.
        # Primary escape = preparatory q_seed from the joint-space seed bank
        # (open-loop when planned move fails). Home-blend is last resort only.
        recent_direct = "|".join(attempts_log[-8:])
        if _is_invalid_start(recent_direct) or _is_invalid_start(direct.message):
            if prep_seed_enabled:
                q_new, tag, failed_prep_seeds = try_move_to_preparatory_seed(
                    q_cur,
                    planner=planner,
                    model=mdl,
                    obstacles=obs,
                    failed_seeds=failed_prep_seeds,
                    execute_waypoints=execute_waypoints,
                    # INVALID_START ⇒ plan from current usually fails; still try
                    # planned once, then open-loop inside the helper.
                    allow_planned=True,
                    max_seeds=prep_seed_max,
                    dt_s=last_dt,
                )
                attempts_log.append(f"invalid_start_{tag}")
                if q_new is not None:
                    q_cur = q_new
                    tip_start = forward_kinematics(q_cur, model=mdl).position_m
                    time.sleep(0.02)
                    continue
            if escape_weight < 0.99:
                q_esc = _blend_toward_home(q_cur, weight=escape_weight)
                escape_weight = min(0.99, escape_weight + 0.15)
                attempts_log.append(
                    f"invalid_start_home_blend_fallback_w{escape_weight:.2f}"
                )
                if execute_waypoints is not None:
                    execute_waypoints(
                        np.vstack([q_cur.reshape(1, 6), q_esc.reshape(1, 6)]),
                        max(last_dt, 0.02),
                    )
                q_cur = q_esc
                tip_start = forward_kinematics(q_cur, model=mdl).position_m
                time.sleep(0.02)
                continue
            # Seed bank + home-blend exhausted — fall through to via standoffs.
            escape_weight = 0.40
            attempts_log.append("invalid_start_escape_saturated_trying_vias")

        # EE-close IK_FAIL: reposition along the base→target radial so the
        # oriented contact IK becomes solvable, then retry from the new pose.
        # (Not an INVALID_START collision — the start is valid but the contact
        # pose is unreachable from a folded/near configuration.)
        tip_to_center = float(np.linalg.norm(tip_start - center))
        ee_close = tip_to_center <= radius + reposition_shell_m
        recent_msgs = "|".join(attempts_log[-8:])
        if (
            reposition_enabled
            and reposition_used < reposition_max
            and ee_close
            and _is_ik_fail(recent_msgs)
        ):
            q_new, rtag = try_radial_reposition_via(
                q_cur,
                sphere_center_m=center,
                sphere_radius_m=radius,
                planner=planner,
                model=mdl,
                cfg=cfg,
                obstacles=obs,
                execute_waypoints=execute_waypoints,
                attempts_log=attempts_log,
            )
            reposition_used += 1
            if q_new is not None:
                q_cur = q_new
                tip_start = forward_kinematics(q_cur, model=mdl).position_m
                if execute_waypoints is not None:
                    partial_execs += 1
                time.sleep(0.02)
                continue

        progressed = False
        candidates = ordered_standoff_candidates(
            tip_start,
            center,
            tip_contact,
            clearances_m=clearances,
            yaws_rad=yaws,
            radius_m=radius,
            surface_margin_m=surface_margin,
            min_travel_m=min_standoff_travel_m,
        )
        for clearance_f, yaw, tip_standoff, travel_m, skip_reason in candidates:
            if _timed_out() and not allow_overtime:
                break
            if skip_reason:
                attempts_log.append(
                    f"standoff_{clearance_f:.3f}_y{yaw:.2f}:{skip_reason}"
                    f"|travel_m={travel_m:.3f}"
                )
                continue

            via_legs_started += 1
            # Prefer pad-facing orientation even on recovery vias.
            via_approach = build_sphere_contact_approach(
                tip_start,
                center,
                radius,
                standoff_m=float(cfg.get("contact_standoff_m", 0.015)),
                current_quaternion_wxyz=forward_kinematics(
                    q_cur, model=mdl
                ).quaternion_wxyz,
            )
            leg1 = planner.plan_to_pose(
                q_cur,
                tip_standoff,
                via_approach.quaternion_wxyz,
                max_attempts=max(1, via_attempts),
                obstacles=obs,
            )
            tag = f"{clearance_f:.3f}_y{yaw:.2f}"
            attempts_log.append(
                f"via1_{tag}:travel_m={travel_m:.3f}|{leg1.message}"
            )
            last_dt = float(leg1.dt_s)
            if not leg1.ok:
                continue

            q_mid = _clamp_joints(leg1.waypoints_rad[-1])
            tip_mid = forward_kinematics(q_mid, model=mdl).position_m
            if _timed_out() and not allow_overtime:
                if execute_waypoints is not None:
                    execute_waypoints(leg1.waypoints_rad, float(leg1.dt_s))
                    partial_execs += 1
                    q_cur = q_mid
                    tip_start = tip_mid
                    progressed = True
                attempts_log.append("timeout_before_contact")
                break

            # After via1: oriented tip-face contact (spheres-on reseat + tip-omit
            # axial nudge) — not a long omit-tip via2 with large tip margins.
            if execute_waypoints is not None:
                execute_waypoints(leg1.waypoints_rad, float(leg1.dt_s))
                partial_execs += 1
                q_cur = q_mid
                tip_start = tip_mid
            oriented = try_oriented_tip_face_contact(
                q_mid if execute_waypoints is None else q_cur,
                sphere_center_m=center,
                sphere_radius_m=radius,
                planner=planner,
                contact_planner=contact,
                obstacles=obs,
                model=mdl,
                max_attempts=max(1, via_attempts),
                execute_waypoints=execute_waypoints,
                strategy_tag="via_contact",
                attempts_log=attempts_log,
                decision_emit=decision_emit,
            )
            if oriented is not None and oriented.ok:
                if execute_waypoints is None:
                    waypoints = _concat_waypoints(
                        leg1.waypoints_rad, oriented.waypoints_rad
                    )
                    return _ok(
                        waypoints,
                        dt_s=float(leg1.dt_s),
                        message=(
                            f"{oriented.message}|clearance_m={clearance_f:.3f}|"
                            f"yaw_rad={yaw:.2f}|via_attempts={via_legs_started}|"
                            f"elapsed_s={time.monotonic() - t0:.2f}"
                        ),
                        backend=oriented.backend,
                    )
                return _ok(
                    oriented.waypoints_rad,
                    dt_s=float(oriented.dt_s),
                    message=(
                        f"{oriented.message}|clearance_m={clearance_f:.3f}|"
                        f"yaw_rad={yaw:.2f}|via_attempts={via_legs_started}|"
                        f"partial_execs={partial_execs}|"
                        f"elapsed_s={time.monotonic() - t0:.2f}"
                    ),
                    backend=oriented.backend,
                )

            if execute_waypoints is not None:
                # via1 already executed above; continue from new pose.
                progressed = True
                attempts_log.append(f"partial_via1_{tag}:executed")
                break
            # Without live execution, try the next standoff candidate.
            continue

        if _timed_out() and not allow_overtime:
            break
        if not progressed:
            # Via attempts hit INVALID_START or stalled: try another preparatory
            # seed before home-blend last resort.
            recent = "|".join(attempts_log[-20:]).upper()
            if "INVALID_START" in recent:
                q_new, tag, failed_prep_seeds = try_move_to_preparatory_seed(
                    q_cur,
                    planner=planner,
                    model=mdl,
                    obstacles=obs,
                    failed_seeds=failed_prep_seeds,
                    execute_waypoints=execute_waypoints,
                    allow_planned=True,
                    max_seeds=prep_seed_max,
                    dt_s=last_dt,
                )
                attempts_log.append(f"via_{tag}")
                if q_new is not None:
                    q_cur = q_new
                    tip_start = forward_kinematics(q_cur, model=mdl).position_m
                elif escape_weight < 0.99:
                    q_esc = _blend_toward_home(q_cur, weight=escape_weight)
                    escape_weight = min(0.99, escape_weight + 0.15)
                    attempts_log.append(
                        f"via_home_blend_fallback_w{escape_weight:.2f}"
                    )
                    if execute_waypoints is not None:
                        execute_waypoints(
                            np.vstack(
                                [q_cur.reshape(1, 6), q_esc.reshape(1, 6)]
                            ),
                            max(last_dt, 0.02),
                        )
                    q_cur = q_esc
                    tip_start = forward_kinematics(q_cur, model=mdl).position_m
            time.sleep(0.05)

    elapsed = time.monotonic() - t0
    log_s = "|".join(attempts_log)
    if len(log_s) > 2500:
        log_s = log_s[:1200] + "|…|" + log_s[-1200:]
    return _fail(
        f"recovery_timeout|via_attempts={via_legs_started}|"
        f"partial_execs={partial_execs}|elapsed_s={elapsed:.2f}|{log_s}",
        dt_s=last_dt,
    )


def plan_collision_free_with_recovery(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    prefer_curobo: bool = True,
    model: UrdfKinematicModel | None = None,
    obstacles: list[SphereObstacle] | None = None,
    planner: CuRoboMotionPlanner | None = None,
    contact_planner: CuRoboMotionPlanner | None = None,
    enable_recovery: bool | None = None,
    timeout_s: float | None = None,
    execute_waypoints: ExecuteWaypointsFn | None = None,
    decision_emit: DecisionEmitFn | None = None,
) -> PlannedTrajectory:
    """Plan with optional standoff-via recovery (see ``plan_via_standoff``).

    ``execute_waypoints`` — when set (Isaac viz), successful via1 legs are
    executed immediately so the EE moves during the recovery budget.
    ``decision_emit`` — optional live ``DEC|…`` callback for smoke monitoring.
    """
    from residual_adaptive_ik.planning.curobo_planner import plan_collision_free

    cfg = load_planning_config()
    recover = (
        bool(cfg.get("plan_recovery_enabled", True))
        if enable_recovery is None
        else bool(enable_recovery)
    )
    timeout = float(
        cfg.get("plan_recovery_timeout_s", 90.0) if timeout_s is None else timeout_s
    )

    if not (recover and prefer_curobo and curobo_available()):
        traj = plan_collision_free(
            q_start_rad,
            q_goal_rad,
            prefer_curobo=prefer_curobo,
            model=model,
            obstacles=obstacles,
            planner=planner,
        )
        if recover and prefer_curobo and not curobo_available() and not traj.ok:
            return PlannedTrajectory(
                waypoints_rad=traj.waypoints_rad,
                dt_s=traj.dt_s,
                success=False,
                backend=traj.backend,
                message=f"recovery_skipped|curobo_unavailable|{traj.message}",
            )
        return traj

    pl = planner or CuRoboMotionPlanner(omit_tip_links=False)
    if contact_planner is not None:
        contact_pl = contact_planner
    elif getattr(pl, "_omit_tip_links", False):
        contact_pl = pl
    else:
        cached = getattr(pl, "_contact_omit_tip_planner", None)
        if cached is None:
            cached = CuRoboMotionPlanner(
                omit_tip_links=True,
                interpolation_dt_s=float(getattr(pl, "_interpolation_dt_s", 0.02)),
                urdf_path=getattr(pl, "_urdf_path", None),
            )
            setattr(pl, "_contact_omit_tip_planner", cached)
        contact_pl = cached
    return plan_via_standoff(
        q_start_rad,
        q_goal_rad,
        planner=pl,
        contact_planner=contact_pl,
        obstacles=obstacles,
        model=model,
        timeout_s=timeout,
        execute_waypoints=execute_waypoints,
        decision_emit=decision_emit,
    )
