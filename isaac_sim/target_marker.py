# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""IK target sphere constants and contact helpers (no Isaac Kit required).

Phase 1–2 viz places a sphere of radius ``TARGET_MARKER_RADIUS_M`` at each IK
goal:

* **red** — pending approach (plan OK, tip not in contact yet)
* **green** — EE tip on/inside the sphere surface (within radius + small outer tol)
* **yellow** — path planning failed; arm must not move (fail closed)

Phase 2 planning treats the same radius as a **volumetric** obstacle: fitted
EE/arm collision spheres must not intersect the marker on the path. Tip contact
at the goal remains allowed when a plan succeeds. See ``spec.md`` Phase 2,
``isaac_sim/viz_plan_policy.py``, and ``isaac_sim/run_ik_viz.py``.
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

# Tip-to-center distance ≤ sphere radius ⇒ tip on or inside the marker surface.
TARGET_MARKER_CONTACT_DISTANCE_M = TARGET_MARKER_RADIUS_M
# Allow a thin outer shell so sim lag / servo error still counts as surface contact.
TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M = 0.003


def marker_rgb_for_state(
    state: MarkerVisualState,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (diffuse, emissive) RGB for a marker visual state."""
    if state is MarkerVisualState.CONTACT:
        return TARGET_MARKER_COLOR_GREEN_RGB, TARGET_MARKER_EMISSIVE_GREEN_RGB
    if state is MarkerVisualState.PLAN_FAIL:
        return TARGET_MARKER_COLOR_YELLOW_RGB, TARGET_MARKER_EMISSIVE_YELLOW_RGB
    return TARGET_MARKER_COLOR_RED_RGB, TARGET_MARKER_EMISSIVE_RED_RGB


def ee_contacts_target(
    ee_position_m: np.ndarray,
    target_position_m: np.ndarray,
    *,
    contact_distance_m: float = TARGET_MARKER_CONTACT_DISTANCE_M,
    outer_tol_m: float = TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
) -> bool:
    """Return True when EE tip contacts the marker surface (or is inside).

    Units: meters. Green when tip-to-center distance is ≤ ``contact_distance_m``
    (sphere radius) plus a small ``outer_tol_m`` for simulation lag. Phase 2
    contact legs plan the tip onto the surface; this switches red → green.
    """
    ee = np.asarray(ee_position_m, dtype=float).reshape(3)
    tgt = np.asarray(target_position_m, dtype=float).reshape(3)
    return float(np.linalg.norm(ee - tgt)) <= float(contact_distance_m) + float(
        outer_tol_m
    )
