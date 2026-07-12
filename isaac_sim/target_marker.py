# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""IK target sphere constants and contact helpers (no Isaac Kit required).

Phase 1 viz places a small sphere at each IK goal. The sphere stays **red**
until the end-effector tip is within ``TARGET_MARKER_CONTACT_DISTANCE_M`` of
the goal center ("contact"), then turns **green**. See ``spec.md`` Phase 1
and ``isaac_sim/run_phase1_ik_viz.py``.
"""
from __future__ import annotations

import numpy as np

# Match v1 ``isaac_lab/mycobot_reach_env.py`` sphere size (meters).
TARGET_MARKER_RADIUS_M = 0.012

# Red = goal not yet reached; green = EE contact with the target sphere.
TARGET_MARKER_COLOR_RED_RGB = (0.95, 0.05, 0.05)
TARGET_MARKER_EMISSIVE_RED_RGB = (0.35, 0.02, 0.02)
TARGET_MARKER_COLOR_GREEN_RGB = (0.10, 0.85, 0.20)
TARGET_MARKER_EMISSIVE_GREEN_RGB = (0.05, 0.35, 0.08)

# Backward-compatible aliases (pre-contact / default highlight).
TARGET_MARKER_COLOR_RGB = TARGET_MARKER_COLOR_RED_RGB
TARGET_MARKER_EMISSIVE_RGB = TARGET_MARKER_EMISSIVE_RED_RGB

# Tip-to-center distance ≤ sphere radius ⇒ tip inside or on the marker volume.
TARGET_MARKER_CONTACT_DISTANCE_M = TARGET_MARKER_RADIUS_M


def ee_contacts_target(
    ee_position_m: np.ndarray,
    target_position_m: np.ndarray,
    *,
    contact_distance_m: float = TARGET_MARKER_CONTACT_DISTANCE_M,
) -> bool:
    """Return True when EE tip is within ``contact_distance_m`` of the target.

    Units: meters. Used to switch the viz sphere from red → green.
    """
    ee = np.asarray(ee_position_m, dtype=float).reshape(3)
    tgt = np.asarray(target_position_m, dtype=float).reshape(3)
    return float(np.linalg.norm(ee - tgt)) <= float(contact_distance_m)
