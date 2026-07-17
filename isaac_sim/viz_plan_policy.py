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

from dataclasses import dataclass, field
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


def skip_unreachable_frac(n_skip_unreachable: int, n_candidates: int) -> float:
    """Fraction of *considered* candidates that were ``SKIPPED_UNREACHABLE``.

    Denominator is candidates inspected while filling countable episodes
    (skips + planned), **not** the countable-episode target. Unreachable
    targets must not inflate the episode count (spec.md Phase 2).
    """
    total = int(n_candidates)
    if total <= 0:
        return 0.0
    return float(n_skip_unreachable) / float(total)


def meets_max_skip_unreachable_frac(
    n_skip_unreachable: int,
    n_candidates: int,
    *,
    max_frac: float,
) -> bool:
    """True when skip-unreachable fraction is ≤ ``max_frac``, or gate disabled.

    ``max_frac < 0`` or ``max_frac >= 1`` disables the gate (always True).
    """
    mf = float(max_frac)
    if mf < 0.0 or mf >= 1.0 - 1e-15:
        return True
    if int(n_candidates) <= 0:
        return True
    return skip_unreachable_frac(n_skip_unreachable, n_candidates) <= mf + 1e-15


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


@dataclass
class ViaPressureTracker:
    r"""Quantify the **repeated need for vias in consecutive episodes**.

    Motivation
    ----------
    When targets are approached from the wrong side of the EE, the direct
    oriented tip-face contact fails and recovery falls back to standoff
    **vias**. A *single* episode needing a via is normal; the diagnostic signal
    is vias being needed **repeatedly, in consecutive episodes** — that points
    to a systematic bad approach angle rather than one awkward pose.

    Mathematical definition (per episode :math:`i`)
    -----------------------------------------------
    Let :math:`a_i \ge 0` be the standoff via count in episode :math:`i` and
    :math:`u_i = \mathbb{1}[a_i \ge 1]` the "needed a via" indicator. With
    smoothing :math:`\alpha \in (0, 1]`:

    - **EMA of via usage** (smoothed fraction of recent episodes needing a via):
      :math:`E_i = \alpha\, u_i + (1-\alpha)\, E_{i-1}`, :math:`E_0 = u_0`.
    - **EMA of via count**:
      :math:`A_i = \alpha\, a_i + (1-\alpha)\, A_{i-1}`.
    - **Consecutive-via streak**:
      :math:`S_i = S_{i-1} + 1` if :math:`u_i = 1` else :math:`0`.

    The pressure is **high** when
    :math:`E_i \ge` ``ema_usage_threshold`` **and**
    :math:`S_i \ge` ``streak_threshold`` — i.e. vias are needed across a
    sustained run of consecutive episodes.

    Units: dimensionless (counts / fractions). Pure + deterministic (testable
    without Isaac).
    """

    alpha: float = 0.5
    ema_usage_threshold: float = 0.6
    streak_threshold: int = 3
    _ema_usage: float | None = field(default=None, repr=False)
    _ema_attempts: float | None = field(default=None, repr=False)
    _streak: int = field(default=0, repr=False)
    _max_streak: int = field(default=0, repr=False)
    _n: int = field(default=0, repr=False)

    def update(self, via_attempts: int) -> dict:
        """Fold one episode's ``via_attempts`` in; return the current signals."""
        a = max(0, int(via_attempts))
        u = 1.0 if a >= 1 else 0.0
        if self._ema_usage is None:
            self._ema_usage = u
            self._ema_attempts = float(a)
        else:
            self._ema_usage = self.alpha * u + (1.0 - self.alpha) * self._ema_usage
            self._ema_attempts = (
                self.alpha * float(a) + (1.0 - self.alpha) * self._ema_attempts
            )
        self._streak = self._streak + 1 if u >= 1.0 else 0
        self._max_streak = max(self._max_streak, self._streak)
        self._n += 1
        high = (
            self._ema_usage >= self.ema_usage_threshold
            and self._streak >= self.streak_threshold
        )
        return {
            "ema_usage": float(self._ema_usage),
            "ema_attempts": float(self._ema_attempts),
            "streak": int(self._streak),
            "max_streak": int(self._max_streak),
            "n": int(self._n),
            "high": bool(high),
        }


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
