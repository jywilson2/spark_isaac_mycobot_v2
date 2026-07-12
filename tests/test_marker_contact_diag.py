# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for marker↔EE contact classification (no CUDA)."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.planning.marker_contact_diag import (
    analyze_waypoint_spheres,
    classify_sphere_vs_marker,
)


def test_tip_sphere_classified_as_tip():
    tip = np.array([0.2, 0.0, 0.15])
    marker = tip.copy()
    # Sphere almost at tip, intersecting 12 mm marker.
    hit = classify_sphere_vs_marker(
        tip + np.array([0.002, 0.0, 0.0]),
        0.006,
        tip,
        marker,
        0.012,
        tip_zone_m=0.028,
        lateral_side_m=0.010,
    )
    assert hit is not None
    assert hit.kind == "tip"


def test_lateral_sphere_classified_as_side():
    tip = np.array([0.2, 0.0, 0.15])
    marker = tip.copy()
    # Sphere beside the tip (EE side), still overlapping marker volume.
    hit = classify_sphere_vs_marker(
        tip + np.array([0.0, 0.018, 0.0]),
        0.008,
        tip,
        marker,
        0.012,
        tip_zone_m=0.028,
        lateral_side_m=0.010,
    )
    assert hit is not None
    assert hit.kind == "side"
    assert hit.lateral_m > 0.010


def test_analyze_waypoint_counts_side_and_tip():
    tip = np.zeros(3)
    marker = np.zeros(3)
    spheres = np.array(
        [
            [0.002, 0.0, 0.0, 0.006],  # tip
            [0.0, 0.018, 0.0, 0.008],  # side
            [1.0, 0.0, 0.0, 0.005],  # far — no hit
        ]
    )
    report = analyze_waypoint_spheres(spheres, tip, marker, marker_radius_m=0.012)
    assert report.n_tip_hits == 1
    assert report.n_side_hits == 1
    assert report.has_side_contact
