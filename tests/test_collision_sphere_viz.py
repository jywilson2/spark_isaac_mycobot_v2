# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for collision-sphere world transforms (no Isaac Kit)."""
from __future__ import annotations

import numpy as np
import pytest

from residual_adaptive_ik.kinematics.urdf_model import get_default_model
from residual_adaptive_ik.planning.collision_sphere_world import (
    collision_spheres_in_base_frame,
    tip_link_names,
)
from residual_adaptive_ik.planning.sphere_fit_mycobot import load_collision_spheres_yaml


def test_link_transforms_include_base_and_ee():
    model = get_default_model()
    q = np.zeros(len(model.revolute_names), dtype=float)
    T = model.link_transforms(q)
    assert model.base_link in T
    assert model.ee_link in T
    np.testing.assert_allclose(T[model.base_link], np.eye(4), atol=1e-12)
    # EE transform matches forward FK tip.
    T_ee, _ = model.forward_transforms(q)
    np.testing.assert_allclose(T[model.ee_link], T_ee, atol=1e-9)


def test_collision_spheres_in_base_frame_nonzero_count():
    data = load_collision_spheres_yaml()
    model = get_default_model()
    q = np.zeros(len(model.revolute_names), dtype=float)
    spheres = collision_spheres_in_base_frame(q, model=model, spheres_data=data)
    assert len(spheres) >= 20
    tips = tip_link_names(data)
    assert any(s.is_tip for s in spheres)
    assert all((s.link in tips) == s.is_tip for s in spheres)
    # All finite, positive radii.
    for s in spheres:
        assert np.all(np.isfinite(s.center_m))
        assert s.radius_m > 0.0


def test_collision_spheres_move_when_joints_change():
    model = get_default_model()
    q0 = np.zeros(len(model.revolute_names), dtype=float)
    q1 = q0.copy()
    q1[1] = 0.4  # shoulder pitch
    s0 = collision_spheres_in_base_frame(q0, model=model)
    s1 = collision_spheres_in_base_frame(q1, model=model)
    assert len(s0) == len(s1)
    # Distal spheres should move; base-link spheres stay put.
    base = [a for a in s0 if a.link == model.base_link]
    base1 = [a for a in s1 if a.link == model.base_link]
    if base and base1:
        np.testing.assert_allclose(base[0].center_m, base1[0].center_m, atol=1e-9)
    distal0 = [a for a in s0 if a.link == model.ee_link]
    distal1 = [a for a in s1 if a.link == model.ee_link]
    assert distal0 and distal1
    assert float(np.linalg.norm(distal0[0].center_m - distal1[0].center_m)) > 1e-3


def test_home_pose_spheres_above_ground_z():
    """Sanity: at zero config, sphere centers are mostly above the table plane."""
    model = get_default_model()
    q = np.zeros(len(model.revolute_names), dtype=float)
    spheres = collision_spheres_in_base_frame(q, model=model)
    zs = np.array([s.center_m[2] for s in spheres])
    assert float(np.median(zs)) > -0.05
