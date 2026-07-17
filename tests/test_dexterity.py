# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Tests for the dexterity prescreen (spec.md Phase 2 step #1).

Verifies the deterministic SKIPPED_UNREACHABLE decision: reachable targets are
feasible; far/out-of-envelope targets are classified unreachable; the region
test and log tokens are correct. Pure CPU (no Isaac/cuRobo).
"""
from __future__ import annotations

import numpy as np

from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import (
    DampedLeastSquaresIK,
    load_joint_limits_rad,
)
from residual_adaptive_ik.planning.dexterity import (
    DexterityResult,
    contact_pose_is_dexterous,
    is_in_dexterous_region,
    load_reach_envelope_m,
    target_radial_distance_m,
)


def test_reach_envelope_loads() -> None:
    lo, hi = load_reach_envelope_m()
    assert 0.0 < lo < hi <= 0.5


def test_radial_distance() -> None:
    d = target_radial_distance_m([0.1, 0.0, 0.0])
    assert abs(d - 0.1) < 1e-9


def test_in_dexterous_region_band() -> None:
    ok, reason = is_in_dexterous_region(
        [0.2, 0.0, 0.0], min_reach_m=0.12, max_reach_m=0.28, margin_m=0.02
    )
    assert ok, reason
    # Just inside min reach -> outside interior shell.
    ok2, _ = is_in_dexterous_region(
        [0.13, 0.0, 0.0], min_reach_m=0.12, max_reach_m=0.28, margin_m=0.02
    )
    assert not ok2
    # Beyond max reach.
    ok3, _ = is_in_dexterous_region(
        [0.5, 0.0, 0.0], min_reach_m=0.12, max_reach_m=0.28, margin_m=0.02
    )
    assert not ok3


def test_far_target_is_unreachable() -> None:
    """A target well beyond max reach must be infeasible (geometric gate)."""
    res = contact_pose_is_dexterous([0.6, 0.0, 0.1], 0.02, n_seeds=6, backend="numpy")
    assert isinstance(res, DexterityResult)
    assert not res.feasible
    assert res.classification == "outside_dexterous_region"
    assert not res.in_region


def test_outside_dexterous_region_skipped_even_if_pose_ik_would_pass() -> None:
    """Near-envelope targets outside the interior shell are skipped geometrically."""
    # max_reach=0.28, margin=0.04 → hi=0.24; 0.26 is outside region.
    res = contact_pose_is_dexterous(
        [0.26, 0.0, 0.0],
        0.02,
        n_seeds=4,
        backend="numpy",
        region_margin_m=0.04,
        min_reach_m=0.12,
        max_reach_m=0.28,
    )
    assert not res.feasible
    assert res.classification == "outside_dexterous_region"


def _find_in_band_fk_tip(lo, hi, *, band=(0.16, 0.24)):
    """Deterministically find a joint config whose FK tip is in a comfy band."""
    rng = np.random.default_rng(7)
    best = None
    for _ in range(400):
        q = rng.uniform(lo, hi)
        tip = forward_kinematics(q).position_m
        radial = float(np.linalg.norm(tip))
        if band[0] < radial < band[1]:
            return tip, radial
        # keep a fallback closest-to-mid-band candidate
        mid = 0.5 * (band[0] + band[1])
        if best is None or abs(radial - mid) < best[2]:
            best = (tip, radial, abs(radial - mid))
    return best[0], best[1]


def test_reachable_in_band_target_not_skipped_by_default() -> None:
    """A genuinely reachable in-band target must NOT be skipped by default.

    Honesty guard: the default prescreen (``skip_orientation_infeasible=False``)
    must never mark an in-region target infeasible — even if the DLS orientation
    oracle stalls — because over-skipping would dishonestly inflate the gate.
    """
    lo, hi = load_joint_limits_rad()
    tip, radial = _find_in_band_fk_tip(lo, hi)
    res = contact_pose_is_dexterous(tip, 0.02, n_seeds=16, backend="numpy")
    assert res.feasible, (radial, res.classification, res.note)
    assert res.classification in ("ok", "unreachable_orientation")
    # Position must be recognized as reachable for an in-band FK-derived target.
    assert res.classification != "unreachable_position"


def test_orientation_skip_flag_semantics() -> None:
    """With the opt-in flag, a position-reachable/orientation-hard target skips.

    Finds an in-region target the DLS oracle cannot orient to, then checks the
    flag flips only that class (position-reachable) to infeasible.
    """
    lo, hi = load_joint_limits_rad()
    rng = np.random.default_rng(1)
    found = None
    for _ in range(200):
        q = rng.uniform(lo, hi)
        tip = forward_kinematics(q).position_m
        if not (0.15 < float(np.linalg.norm(tip)) < 0.26):
            continue
        res = contact_pose_is_dexterous(tip, 0.02, n_seeds=10, backend="numpy")
        if res.classification == "unreachable_orientation":
            found = tip
            break
    if found is None:
        import pytest

        pytest.skip("no orientation-limited in-region target found in search")
    default = contact_pose_is_dexterous(found, 0.02, n_seeds=10, backend="numpy")
    assert default.feasible is True  # NumPy default never skips orientation-limited
    opted = contact_pose_is_dexterous(
        found, 0.02, n_seeds=10, backend="numpy", skip_orientation_infeasible=True
    )
    assert opted.feasible is False
    assert opted.classification == "unreachable_orientation"


def test_log_tokens_are_parseable_shape() -> None:
    res = contact_pose_is_dexterous([0.6, 0.0, 0.1], 0.02, n_seeds=4, backend="numpy")
    tokens = res.as_log_tokens()
    for key in ("reason=", "in_region=", "radial_m=", "pos_err_m=",
                "ori_err_rad=", "orientations=", "axis=", "backend="):
        assert key in tokens


def test_prescreen_is_deterministic() -> None:
    a = contact_pose_is_dexterous([0.45, 0.1, 0.2], 0.02, n_seeds=8)
    b = contact_pose_is_dexterous([0.45, 0.1, 0.2], 0.02, n_seeds=8)
    assert a.feasible == b.feasible
    assert a.classification == b.classification
    assert abs(a.radial_m - b.radial_m) < 1e-12
