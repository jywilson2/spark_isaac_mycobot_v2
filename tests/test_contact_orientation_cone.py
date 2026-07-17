# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Tests for the bounded contact-orientation cone (spec.md Phase 2 step #2)."""
from __future__ import annotations

import numpy as np
import pytest

from residual_adaptive_ik.planning.contact_geometry import (
    contact_orientation_cone,
    quaternion_facing_sphere,
    tool_axis_alignment_error_rad,
)


def test_cone_returns_exact_normal_first() -> None:
    normal = np.array([0.0, 0.0, 1.0])
    quats = contact_orientation_cone(normal, cone_max_rad=0.3, n_tilts=2, n_azimuths=4)
    exact = quaternion_facing_sphere(normal)
    assert np.allclose(quats[0], exact)
    # 1 exact + n_tilts * n_azimuths cone entries.
    assert len(quats) == 1 + 2 * 4


def test_cone_stays_within_tolerance_of_normal() -> None:
    """Every cone tool axis must be within cone_max_rad of the outward normal."""
    normal = np.array([0.3, -0.4, 0.86])
    normal = normal / np.linalg.norm(normal)
    cone_max = 0.3
    quats = contact_orientation_cone(
        normal, cone_max_rad=cone_max, n_tilts=3, n_azimuths=6
    )
    for q in quats:
        err = tool_axis_alignment_error_rad(q, normal)
        assert err <= cone_max + 1e-6


def test_cone_stays_under_honest_gate_tolerance() -> None:
    """A ~17deg cone must not exceed the 35deg tip-face gate tolerance."""
    from isaac_sim.target_marker import TARGET_MARKER_TOOL_AXIS_TOL_RAD

    normal = np.array([1.0, 0.2, 0.1])
    normal = normal / np.linalg.norm(normal)
    quats = contact_orientation_cone(normal, cone_max_rad=0.30, n_tilts=2, n_azimuths=4)
    for q in quats:
        assert (
            tool_axis_alignment_error_rad(q, normal)
            < TARGET_MARKER_TOOL_AXIS_TOL_RAD
        )


def test_cone_disabled_when_zero_span() -> None:
    normal = np.array([0.0, 1.0, 0.0])
    quats = contact_orientation_cone(normal, cone_max_rad=0.0, n_tilts=2, n_azimuths=4)
    assert len(quats) == 1


def test_cone_is_deterministic() -> None:
    normal = np.array([0.1, 0.2, 0.97])
    a = contact_orientation_cone(normal, cone_max_rad=0.25, n_tilts=2, n_azimuths=5)
    b = contact_orientation_cone(normal, cone_max_rad=0.25, n_tilts=2, n_azimuths=5)
    assert len(a) == len(b)
    for qa, qb in zip(a, b):
        assert np.allclose(qa, qb)
