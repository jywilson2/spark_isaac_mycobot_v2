# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Even workspace target distribution (v1 stratified envelope)."""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.kinematics.baseline_eval import (
    evaluate_baseline,
    sample_stratified_target_poses,
)
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.workspace_sampling import (
    bin_occupancy_counts,
    load_workspace_config,
    sample_stratified_workspace_positions,
)


def test_workspace_config_matches_vendor_reach():
    env = load_workspace_config()
    assert env.max_reach_m == 0.280
    assert env.min_reach_m == 0.12
    assert env.max_joint_speed_deg_s == 160.0
    assert env.n_bins == 12 * 4 * 5


def test_stratified_samples_cover_all_bins_when_n_equals_nbins():
    env = load_workspace_config()
    rng = np.random.default_rng(0)
    pts = sample_stratified_workspace_positions(env.n_bins, rng, env)
    counts = bin_occupancy_counts(pts, env)
    assert counts.shape == (env.n_bins,)
    assert int(np.min(counts)) >= 1
    assert int(np.sum(counts)) == env.n_bins


def test_stratified_samples_are_even_for_large_n():
    """With many samples, max/min bin occupancy ratio stays bounded."""
    env = load_workspace_config()
    rng = np.random.default_rng(1)
    n = env.n_bins * 10
    pts = sample_stratified_workspace_positions(n, rng, env)
    counts = bin_occupancy_counts(pts, env)
    assert int(np.sum(counts)) == n
    # Round-robin → every bin gets exactly 10
    assert int(np.min(counts)) == 10
    assert int(np.max(counts)) == 10


def test_stratified_points_lie_in_envelope():
    env = load_workspace_config()
    rng = np.random.default_rng(2)
    pts = sample_stratified_workspace_positions(200, rng, env)
    radii = np.hypot(pts[:, 0], pts[:, 1])
    assert np.all(radii >= env.min_reach_m - 1e-9)
    assert np.all(radii <= env.max_reach_m + 1e-9)
    assert np.all(pts[:, 2] >= env.min_z_m - 1e-9)
    assert np.all(pts[:, 2] <= env.max_z_m + 1e-9)


def test_baseline_default_sampling_is_stratified_and_covers_bins():
    env = load_workspace_config()
    metrics, trials = evaluate_baseline(
        n_poses=env.n_bins,  # one target per stratified cell
        seed=0,
        return_trials=True,
        sampling="stratified",
    )
    assert metrics["sampling"] == "stratified"
    assert len(trials) == env.n_bins
    positions = np.stack([t.target.position_m for t in trials], axis=0)
    counts = bin_occupancy_counts(positions, env)
    assert int(np.min(counts)) >= 1
    assert int(np.sum(counts)) == env.n_bins


def test_sample_stratified_target_poses_shape():
    model = load_urdf_model()
    lo, hi = load_joint_limits_rad()
    rng = np.random.default_rng(3)
    poses, q_true = sample_stratified_target_poses(
        n=24, rng=rng, model=model, joint_lower_rad=lo, joint_upper_rad=hi
    )
    assert len(poses) == 24
    assert q_true.shape == (24, 6)
