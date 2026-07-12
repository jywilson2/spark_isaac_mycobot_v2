# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Kit-free viz / planning policy helpers (testable without Isaac Sim).

Why this module exists
----------------------
A past bug returned ``PLAN_OK`` via NumPy ``ok_fallback|curobo=plan_failed:...``
and the GUI still moved the arm into the marker. These helpers encode the
fail-closed execution policy and marker color state so unit tests can catch
regressions without Kit.

See ``spec.md`` Phase 2, ``configs/planning/collision.yaml``, and
``isaac_sim/run_ik_viz.py``.
"""
from __future__ import annotations

from enum import Enum
from typing import Any


class MarkerVisualState(str, Enum):
    """IK target sphere appearance in Phase 1–2 viz."""

    PENDING = "pending"  # red — awaiting approach / contact
    CONTACT = "contact"  # green — EE tip in marker volume
    PLAN_FAIL = "plan_fail"  # yellow — path planning rejected; no motion


def may_execute_motion(plan_ok: bool, *, gate_on_failure: bool = True) -> bool:
    """Return True only when joint motion toward the IK goal is allowed.

    Fail closed: a failed plan never executes, even if ``gate_on_failure`` is
    False. (Ungated IK lerp was the source of marker collisions.)
    """
    del gate_on_failure  # retained for call-site clarity / future policy knobs
    return bool(plan_ok)


def plan_ok_rate(n_ok: int, n_fail: int) -> float:
    """Return PLAN_OK fraction in ``[0, 1]`` (0 if no planning trials)."""
    total = int(n_ok) + int(n_fail)
    if total <= 0:
        return 0.0
    return float(n_ok) / float(total)


def meets_min_plan_ok_rate(
    n_ok: int,
    n_fail: int,
    *,
    min_rate: float,
) -> bool:
    """True when planning success rate meets ``min_rate``, or no trials ran."""
    total = int(n_ok) + int(n_fail)
    if total <= 0:
        return True
    return plan_ok_rate(n_ok, n_fail) + 1e-15 >= float(min_rate)


def plan_result_is_executable(traj: Any) -> bool:
    """Return True when a ``PlannedTrajectory``-like object may drive the arm.

    Rejects:
    - ``success=False`` / empty waypoints
    - historical unsafe pattern: success with ``ok_fallback`` + ``plan_failed``
    """
    ok = bool(getattr(traj, "ok", False))
    if not ok:
        return False
    waypoints = getattr(traj, "waypoints_rad", None)
    if waypoints is None:
        return False
    try:
        size = int(getattr(waypoints, "size"))
    except Exception:
        size = len(waypoints)
    if size <= 0:
        return False
    msg = str(getattr(traj, "message", "") or "").lower()
    if "ok_fallback" in msg and "plan_failed" in msg:
        return False
    if "rejected_no_numpy_fallback" in msg:
        return False
    # Recovery may have already executed vias via execute_waypoints; a 1-row
    # hold at the final joints is still executable.
    return True


def resolve_marker_visual_state(
    *,
    plan_ok: bool,
    tip_contacted: bool,
) -> MarkerVisualState:
    """Map plan + tip contact to sphere color state.

    Priority: plan failure → yellow; else tip contact → green; else red.
    """
    if not plan_ok:
        return MarkerVisualState.PLAN_FAIL
    if tip_contacted:
        return MarkerVisualState.CONTACT
    return MarkerVisualState.PENDING
