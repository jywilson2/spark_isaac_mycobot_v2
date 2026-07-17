# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""IK target sphere constants and contact helpers (no Isaac Kit required).

Phase 1–2 viz places a sphere of radius ``TARGET_MARKER_RADIUS_M`` at each IK
goal:

* **red** — pending approach (plan OK, tip not in tip-face contact yet)
* **green** — EE **tip-face center** on the marker surface along the approach
  ray (side / flange grazes do **not** count)
* **yellow** — path planning failed after recovery timeout; arm must not move

Phase 2 planning treats the same radius as a **volumetric** obstacle: fitted
EE/arm collision spheres must not intersect the marker on the path. See
``spec.md`` Phase 2, ``isaac_sim/viz_plan_policy.py``, and
``isaac_sim/run_ik_viz.py``.
"""
from __future__ import annotations

import numpy as np

from isaac_sim.viz_plan_policy import MarkerVisualState

# Match v1 ``isaac_lab/mycobot_reach_env.py`` sphere size (meters).
TARGET_MARKER_RADIUS_M = 0.012

# Red = goal not yet reached; green = EE contact; yellow = plan failed.
TARGET_MARKER_COLOR_RED_RGB = (0.95, 0.05, 0.05)
TARGET_MARKER_EMISSIVE_RED_RGB = (0.35, 0.02, 0.02)
TARGET_MARKER_COLOR_GREEN_RGB = (0.10, 0.85, 0.20)
TARGET_MARKER_EMISSIVE_GREEN_RGB = (0.05, 0.35, 0.08)
TARGET_MARKER_COLOR_YELLOW_RGB = (0.95, 0.85, 0.10)
TARGET_MARKER_EMISSIVE_YELLOW_RGB = (0.40, 0.35, 0.04)

# Backward-compatible aliases (pre-contact / default highlight).
TARGET_MARKER_COLOR_RGB = TARGET_MARKER_COLOR_RED_RGB
TARGET_MARKER_EMISSIVE_RGB = TARGET_MARKER_EMISSIVE_RED_RGB

# Tip-to-center distance ≤ sphere radius ⇒ tip on or inside the marker volume.
TARGET_MARKER_CONTACT_DISTANCE_M = TARGET_MARKER_RADIUS_M
# Allow an outer shell so Isaac PD servo lag still counts as surface contact.
# 8 mm covers typical shortfall after cuRobo contact-leg plans (was 3 mm).
TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M = 0.008
# Max lateral offset (meters) from the approach axis for valid tip-face contact.
# Rejects pure equator/side grazes (lateral ≈ sphere radius) while allowing
# approaches up to ~56° off the ideal axis (sin(56°) × 12 mm ≈ 10 mm).
TARGET_MARKER_TIP_FACE_RADIUS_M = 0.010
# Max tool-axis misalignment (radians) for a valid tip-face contact, measured
# SIGNED against the OUTWARD normal (tip − center): the flange +Z must point
# out along the approach ray (axis_out ≈ 0°). This rejects both "side of the
# EE" contacts where the sphere touches the barrel (tool axis ⟂ radial,
# axis_out ≈ 90°) AND "flipped/back onto the marker" contacts where +Z points
# toward the center (axis_out ≈ 180°). The sign was established empirically:
# the reachable, visually-correct contacts measure axis_out ≈ 0° (axis_in ≈
# 180°), and commanding the inward sign makes cuRobo IK fail. Being signed
# (not folded to [0, 90°]) is what rejects the flipped case the old gate
# accepted. ~35° allows servo lag; NOTE this planning/FK +Z is the URDF
# joint6_flange local Z (see contact_geometry frame-convention note).
TARGET_MARKER_TOOL_AXIS_TOL_RAD = 0.611  # ≈ 35°
# Penetration tolerance (meters) past the sphere center plane along the
# approach axis. A valid tip-face contact stays on the *near* hemisphere; if
# the tip crosses to the far side (signed axial distance from center > this),
# the EE has passed **through** the marker and must not count as success.
TARGET_MARKER_THROUGH_PENETRATION_TOL_M = 0.003
# Base-column keepout (meters): targets inside this cylinder near the base
# cause cuRobo INVALID_START_STATE_WORLD_COLLISION even from home (mesh-fitted
# spheres overlap the marker OBB). Skip them from the rate gate.
TARGET_MARKER_BASE_KEEPOUT_XY_M = 0.14
TARGET_MARKER_BASE_KEEPOUT_Z_M = 0.16


def marker_rgb_for_state(
    state: MarkerVisualState,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (diffuse, emissive) RGB for a marker visual state."""
    if state is MarkerVisualState.CONTACT:
        return TARGET_MARKER_COLOR_GREEN_RGB, TARGET_MARKER_EMISSIVE_GREEN_RGB
    if state is MarkerVisualState.PLAN_FAIL:
        return TARGET_MARKER_COLOR_YELLOW_RGB, TARGET_MARKER_EMISSIVE_YELLOW_RGB
    return TARGET_MARKER_COLOR_RED_RGB, TARGET_MARKER_EMISSIVE_RED_RGB


def tip_face_pierce_point_m(
    approach_from_m: np.ndarray,
    target_position_m: np.ndarray,
    *,
    radius_m: float = TARGET_MARKER_RADIUS_M,
) -> np.ndarray:
    """Ideal tip-face contact point on the near sphere surface (meters).

    Along the ray from ``approach_from_m`` toward the marker center, at
    distance ``radius_m`` from the center (surface, approach side).
    """
    start = np.asarray(approach_from_m, dtype=float).reshape(3)
    center = np.asarray(target_position_m, dtype=float).reshape(3)
    delta = center - start
    dist = float(np.linalg.norm(delta))
    r = float(radius_m)
    if dist < 1e-9:
        return center - np.array([0.0, 0.0, r], dtype=float)
    return center - (r / dist) * delta


def classify_tip_contact(
    ee_position_m: np.ndarray,
    target_position_m: np.ndarray,
    *,
    contact_distance_m: float = TARGET_MARKER_CONTACT_DISTANCE_M,
    outer_tol_m: float = TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
    approach_from_m: np.ndarray | None = None,
    tip_face_radius_m: float = TARGET_MARKER_TIP_FACE_RADIUS_M,
    ee_quaternion_wxyz: np.ndarray | None = None,
    axis_tol_rad: float = TARGET_MARKER_TOOL_AXIS_TOL_RAD,
    through_tol_m: float = TARGET_MARKER_THROUGH_PENETRATION_TOL_M,
) -> tuple[bool, str, dict]:
    """Classify an EE tip sample against the marker (diagnostic + gate core).

    Why this exists
    ---------------
    The bare distance check let two bad approaches count as "green":
    (a) the sphere touches the **side/barrel** of the tool (tool axis ⟂ the
    approach ray) rather than the tip pad, and (b) the tip drives **through**
    the marker to the far hemisphere. Detecting these explicitly is what the
    Phase 2 tip-face contact requirement is about (``spec.md`` tip-face
    planning). This function returns the decision, a human reason, and the
    measured metrics so the viz can log *why* a sample was accepted/rejected.

    Geometry (meters / radians)
    ---------------------------
    Let ``a`` be the unit approach direction (approach_from → center) and
    ``s = dot(ee − center, a)`` the signed axial position of the tip:
    ``s ≈ −radius`` on the near surface, ``0`` at the center, ``+radius`` on
    the far surface. ``penetration_m`` returns ``s``; ``s > through_tol_m``
    means the tip crossed to the far side (passed through).

    The tool-axis error is measured **signed** against the *outward* normal
    (tip − center): a genuine front tip-face contact has the flange +Z along the
    outward approach ray (``axis_out_err`` small). This was established
    empirically — the reachable, visually-correct contacts measure
    ``axis_out ≈ 0`` (``axis_in ≈ π``), and commanding the inward sign to cuRobo
    fails IK. A contact where +Z is ⟂ the radial (``axis_out ≈ π/2``) is the
    marker on the **side/barrel** of the EE, and one where +Z points *toward*
    the center (``axis_out ≈ π``) is the flange **flipped/back** onto the marker;
    both are rejected as ``wrong_side_axis``. Being signed (not folded to
    ``[0, π/2]``) is what distinguishes this from the old gate that wrongly
    accepted the flipped/back contact. See ``TARGET_MARKER_TOOL_AXIS_TOL_RAD``
    and ``docs/phase2_geometry.md``.

    Returns
    -------
    ``(is_contact, reason, metrics)`` where ``reason`` is one of
    ``"ok" | "no_contact" | "through" | "side_graze" | "wrong_side_axis"`` and
    ``metrics`` carries ``dist_m, penetration_m, lateral_m, axis_in_err_rad,
    axis_out_err_rad, axis_line_err_rad``.
    """
    ee = np.asarray(ee_position_m, dtype=float).reshape(3)
    tgt = np.asarray(target_position_m, dtype=float).reshape(3)
    dist = float(np.linalg.norm(ee - tgt))
    metrics: dict = {
        "dist_m": dist,
        "penetration_m": float("nan"),
        "lateral_m": float("nan"),
        "axis_in_err_rad": float("nan"),
        "axis_out_err_rad": float("nan"),
        "axis_line_err_rad": float("nan"),
    }
    if dist > float(contact_distance_m) + float(outer_tol_m):
        return False, "no_contact", metrics

    # Approach axis (start → center). When absent, fall back to the outward
    # radial through the tip so penetration is still measurable.
    v = ee - tgt
    if approach_from_m is not None:
        start = np.asarray(approach_from_m, dtype=float).reshape(3)
        delta = tgt - start
        an = float(np.linalg.norm(delta))
        approach_u = (
            delta / an if an > 1e-9 else np.array([0.0, 0.0, 1.0], dtype=float)
        )
        axial = float(np.dot(v, approach_u))
        metrics["penetration_m"] = axial
        metrics["lateral_m"] = float(np.linalg.norm(v - axial * approach_u))
    else:
        approach_u = None

    if ee_quaternion_wxyz is not None:
        from residual_adaptive_ik.planning.contact_geometry import (
            tool_axis_alignment_error_rad,
        )

        # Outward normal at tip ≈ tip − center (direction from the sphere center
        # out to the tip). A correct front tip-face contact has tool +Z along
        # this outward ray (flange facing back along the approach). When the tip
        # sits on the center, use the approach direction as the reference.
        on = float(np.linalg.norm(v))
        outward = (v / on) if on > 1e-9 else (
            -approach_u if approach_u is not None
            else np.array([0.0, 0.0, 1.0], dtype=float)
        )
        err_out = tool_axis_alignment_error_rad(ee_quaternion_wxyz, outward)
        metrics["axis_out_err_rad"] = float(err_out)
        metrics["axis_in_err_rad"] = float(np.pi - err_out)
        metrics["axis_line_err_rad"] = float(min(err_out, np.pi - err_out))

    # Legacy / unit callers without an approach ray or orientation: volume only.
    if approach_from_m is None and ee_quaternion_wxyz is None:
        return True, "ok", metrics

    if approach_from_m is not None:
        if metrics["penetration_m"] > float(through_tol_m) + 1e-9:
            return False, "through", metrics
        if metrics["lateral_m"] > float(tip_face_radius_m) + 1e-9:
            return False, "side_graze", metrics
    if ee_quaternion_wxyz is not None:
        # SIGNED: require +Z along the outward normal (front tip-face contact).
        # Rejects side/barrel (axis_out ≈ π/2) and flipped/back (axis_out ≈ π).
        if metrics["axis_out_err_rad"] > float(axis_tol_rad) + 1e-9:
            return False, "wrong_side_axis", metrics
    return True, "ok", metrics


def ee_contacts_target(
    ee_position_m: np.ndarray,
    target_position_m: np.ndarray,
    *,
    contact_distance_m: float = TARGET_MARKER_CONTACT_DISTANCE_M,
    outer_tol_m: float = TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
    approach_from_m: np.ndarray | None = None,
    tip_face_radius_m: float = TARGET_MARKER_TIP_FACE_RADIUS_M,
    ee_quaternion_wxyz: np.ndarray | None = None,
    axis_tol_rad: float | None = None,
) -> bool:
    """Return True when the EE **tip-face center** contacts the marker.

    Thin wrapper over :func:`classify_tip_contact`. Units: meters. See that
    function for the full contact requirements (near-surface, non-penetration,
    lateral tip-face, tool-axis collinearity).
    """
    ok, _reason, _metrics = classify_tip_contact(
        ee_position_m,
        target_position_m,
        contact_distance_m=contact_distance_m,
        outer_tol_m=outer_tol_m,
        approach_from_m=approach_from_m,
        tip_face_radius_m=tip_face_radius_m,
        ee_quaternion_wxyz=ee_quaternion_wxyz,
        axis_tol_rad=(
            TARGET_MARKER_TOOL_AXIS_TOL_RAD
            if axis_tol_rad is None
            else float(axis_tol_rad)
        ),
    )
    return ok
