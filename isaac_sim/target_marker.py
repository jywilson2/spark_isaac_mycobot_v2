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


def ee_contacts_target(
    ee_position_m: np.ndarray,
    target_position_m: np.ndarray,
    *,
    contact_distance_m: float = TARGET_MARKER_CONTACT_DISTANCE_M,
    outer_tol_m: float = TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
    approach_from_m: np.ndarray | None = None,
    tip_face_radius_m: float = TARGET_MARKER_TIP_FACE_RADIUS_M,
) -> bool:
    """Return True when the EE **tip-face center** contacts the marker.

    Units: meters.

    Requirements
    ------------
    1. Tip-to-center distance ≤ ``contact_distance_m + outer_tol_m`` (on/near
       the sphere).
    2. When ``approach_from_m`` is set (viz / recovery), the tip's **lateral**
       offset from the approach axis (through the marker center) must be
       ≤ ``tip_face_radius_m``. That is the middle of the EE tip contact pad.
       Side / equator grazes fail; axial immersion along the approach still
       counts.
    """
    ee = np.asarray(ee_position_m, dtype=float).reshape(3)
    tgt = np.asarray(target_position_m, dtype=float).reshape(3)
    dist = float(np.linalg.norm(ee - tgt))
    if dist > float(contact_distance_m) + float(outer_tol_m):
        return False
    if approach_from_m is None:
        # Legacy / unit callers without an approach ray: volume check only.
        return True
    start = np.asarray(approach_from_m, dtype=float).reshape(3)
    delta = tgt - start
    an = float(np.linalg.norm(delta))
    if an < 1e-9:
        approach_u = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        approach_u = delta / an
    # Lateral distance from the approach axis through the marker center.
    v = ee - tgt
    axial = float(np.dot(v, approach_u))
    lateral = float(np.linalg.norm(v - axial * approach_u))
    return lateral <= float(tip_face_radius_m) + 1e-9
