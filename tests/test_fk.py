# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""FK contract tests — Phase 1 acceptance."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from residual_adaptive_ik.kinematics.fk import Pose, forward_kinematics
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model, resolve_urdf_path

REPO = Path(__file__).resolve().parents[1]
KINEMATICS_URDF = REPO / "assets" / "urdf" / "mycobot_280_m5_kinematics.urdf"


def test_resolve_urdf_finds_kinematics_asset():
    path = resolve_urdf_path(KINEMATICS_URDF)
    assert path.is_file()


def test_forward_kinematics_zero_pose_finite():
    pose = forward_kinematics(np.zeros(6), urdf_path=KINEMATICS_URDF)
    assert isinstance(pose, Pose)
    assert pose.position_m.shape == (3,)
    assert pose.quaternion_wxyz.shape == (4,)
    assert np.all(np.isfinite(pose.position_m))
    assert np.all(np.isfinite(pose.quaternion_wxyz))
    assert pytest.approx(1.0, abs=1e-9) == float(np.linalg.norm(pose.quaternion_wxyz))


def test_forward_kinematics_zero_pose_matches_urdf_fixture():
    """Zero configuration flange pose from mycobot_280_m5 joint origins."""
    pose = forward_kinematics(np.zeros(6), urdf_path=KINEMATICS_URDF)
    expected = np.array([0.045600031438863466, -0.06462102695479828, 0.4111397626357216])
    assert np.allclose(pose.position_m, expected, atol=1e-9)
    # Working radius (~0.28 m) is for *reachable targets*, not this stretched zero pose.
    assert float(np.linalg.norm(pose.position_m)) < 0.50


def test_forward_kinematics_rejects_bad_shape():
    with pytest.raises(ValueError):
        forward_kinematics(np.zeros(5), urdf_path=KINEMATICS_URDF)


def test_vendor_and_asset_urdf_agree_when_both_present():
    """If third_party / sibling vendor URDF exists, FK must match the asset strip."""
    model_asset = load_urdf_model(KINEMATICS_URDF)
    try:
        from residual_adaptive_ik.kinematics.urdf_model import default_urdf_candidates

        vendor = next(
            (
                p
                for p in default_urdf_candidates()
                if p.is_file() and p.resolve() != KINEMATICS_URDF.resolve()
            ),
            None,
        )
    except Exception:
        vendor = None
    if vendor is None:
        pytest.skip("vendor mycobot_ros2 URDF not available")
    model_vendor = load_urdf_model(vendor)
    q = np.array([0.1, -0.2, 0.3, -0.1, 0.2, 0.0])
    p_a, q_a = model_asset.forward(q)
    p_v, q_v = model_vendor.forward(q)
    assert np.allclose(p_a, p_v, atol=1e-9)
    # Quaternions may flip sign and remain equivalent
    assert np.allclose(q_a, q_v, atol=1e-9) or np.allclose(q_a, -q_v, atol=1e-9)
