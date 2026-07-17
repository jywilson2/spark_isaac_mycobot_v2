# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for oriented tip-face contact geometry (no CUDA)."""
from __future__ import annotations

import numpy as np
import pytest

from residual_adaptive_ik.planning.contact_geometry import (
    build_sphere_contact_approach,
    tool_axis_alignment_error_rad,
    tool_axis_from_quaternion,
    validate_axial_contact_segment,
)
from residual_adaptive_ik.utils.transforms import quaternion_to_rotation_matrix


def test_build_sphere_contact_approach_pad_faces_sphere():
    tip = np.array([0.20, 0.0, 0.10])
    center = np.array([0.10, 0.0, 0.10])
    radius = 0.012
    ap = build_sphere_contact_approach(
        tip, center, radius, standoff_m=0.015, nudge_m=0.003
    )
    # Pierce on near surface between tip and center.
    assert float(np.linalg.norm(ap.pierce_position_m - center)) == pytest.approx(
        radius, abs=1e-9
    )
    # Standoff further out along the same ray.
    assert float(np.linalg.norm(ap.standoff_position_m - center)) == pytest.approx(
        radius + 0.015, abs=1e-9
    )
    # Planning-frame tool +Z is commanded along the OUTWARD normal — this is
    # the orientation cuRobo can actually reach for a near-side approach
    # (commanding the inward sign makes the contact IK fail). The URDF/FK tool
    # +Z is opposite, so the achieved contact reads as +Z-toward-center, which
    # is what the signed tip-face gate requires.
    z = tool_axis_from_quaternion(ap.quaternion_wxyz)
    assert float(np.dot(z, ap.normal_outward)) == pytest.approx(1.0, abs=1e-6)
    R = quaternion_to_rotation_matrix(ap.quaternion_wxyz)
    assert float(np.linalg.det(R)) == pytest.approx(1.0, abs=1e-6)


def test_validate_axial_contact_segment_accepts_short_inward_move():
    n = np.array([1.0, 0.0, 0.0])
    start = np.array([0.03, 0.0, 0.0])
    end = np.array([0.012, 0.0, 0.0])
    ok, reason = validate_axial_contact_segment(
        start, end, n, max_length_m=0.005, lateral_tol_m=0.002
    )
    # length 0.018 > 0.005 → too long
    assert not ok
    assert "too_long" in reason
    ok2, reason2 = validate_axial_contact_segment(
        start, end, n, max_length_m=0.020, lateral_tol_m=0.002
    )
    assert ok2 and reason2 == "ok"


def test_tool_axis_alignment_error_identity():
    # Identity quat → tool +Z is world +Z.
    q = np.array([1.0, 0.0, 0.0, 0.0])
    err = tool_axis_alignment_error_rad(q, np.array([0.0, 0.0, 1.0]))
    assert err == pytest.approx(0.0, abs=1e-9)
    err90 = tool_axis_alignment_error_rad(q, np.array([1.0, 0.0, 0.0]))
    assert err90 == pytest.approx(np.pi / 2, abs=1e-6)
