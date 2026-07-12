# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Target-marker constants and contact coloring (no Kit required)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from isaac_sim.target_marker import (
    TARGET_MARKER_COLOR_GREEN_RGB,
    TARGET_MARKER_COLOR_RED_RGB,
    TARGET_MARKER_COLOR_YELLOW_RGB,
    TARGET_MARKER_CONTACT_DISTANCE_M,
    TARGET_MARKER_RADIUS_M,
    ee_contacts_target,
    marker_rgb_for_state,
)
from isaac_sim.viz_plan_policy import MarkerVisualState


REPO = Path(__file__).resolve().parents[1]


def test_target_marker_radius_matches_v1():
    """v1 Isaac Lab uses a 12 mm sphere; Phase 1 viz must match size."""
    assert TARGET_MARKER_RADIUS_M == 0.012
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "UsdGeom.Sphere" in src
    assert "ee_contacts_target" in src
    assert "TARGET_MARKER_COLOR_GREEN_RGB" in src or "contacted" in src


def test_ee_contacts_target_red_until_within_sphere():
    """Sphere stays 'not contacted' until tip is within contact distance."""
    target = np.array([0.20, 0.0, 0.15])
    far = target + np.array([0.05, 0.0, 0.0])
    assert not ee_contacts_target(far, target)
    # Slightly inside the sphere (exact radius can fail float equality).
    near = target + np.array([TARGET_MARKER_CONTACT_DISTANCE_M * 0.999, 0.0, 0.0])
    assert ee_contacts_target(near, target)
    inside = target + np.array([TARGET_MARKER_CONTACT_DISTANCE_M * 0.5, 0.0, 0.0])
    assert ee_contacts_target(inside, target)
    just_outside = target + np.array([TARGET_MARKER_CONTACT_DISTANCE_M * 1.001, 0.0, 0.0])
    assert not ee_contacts_target(just_outside, target)


def test_red_and_green_rgb_distinct():
    assert TARGET_MARKER_COLOR_RED_RGB[0] > TARGET_MARKER_COLOR_RED_RGB[1]
    assert TARGET_MARKER_COLOR_GREEN_RGB[1] > TARGET_MARKER_COLOR_GREEN_RGB[0]


def test_yellow_rgb_for_plan_fail():
    assert TARGET_MARKER_COLOR_YELLOW_RGB[0] > 0.8
    assert TARGET_MARKER_COLOR_YELLOW_RGB[1] > 0.7
    assert TARGET_MARKER_COLOR_YELLOW_RGB[2] < 0.3
    assert marker_rgb_for_state(MarkerVisualState.PLAN_FAIL)[0] == TARGET_MARKER_COLOR_YELLOW_RGB


def test_target_marker_avoids_display_color_without_indices():
    """Fabric warns if displayColor is set without primvars:displayColor:indices."""
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert ".CreateDisplayColorAttr(" not in src
    assert "UsdPreviewSurface" in src
    # Phase 2: marker is a volumetric planning obstacle (not visual-only).
    assert "volumetric" in src
    assert "obstacles=[target_obstacle]" in src or "SphereObstacle" in src
    assert "MarkerVisualState.PLAN_FAIL" in src
