# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Motion-planning recovery: standoff via-waypoints with a wall-clock timeout.

Why
---
Direct tip→marker-surface plans often ``IK_FAIL`` when the start pose is awkward
or the EE is already near the volumetric target. Industrial practice splits the
motion into **via** segments: plan to a standoff pose outside the obstacle, then
a short final approach to the contact surface.

Important: recovery retries are **planning** attempts. The arm only moves after a
full plan succeeds. Yellow marker + motionless arm means every strategy failed —
logs must still show ``via1_`` attempts when recovery is enabled.

This is classical motion planning — not residual learning. See
``docs/phase2_geometry.md`` and ``configs/planning/collision.yaml``.

Units: meters, radians, seconds.
"""
from __future__ import annotations

import re
import time
from typing import Sequence

import numpy as np

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel, get_default_model
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    PlannedTrajectory,
    curobo_available,
    load_planning_config,
    tip_on_sphere_surface,
)


def tip_standoff_on_approach(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    *,
    clearance_m: float,
) -> np.ndarray:
    """Tip pose on the approach ray at ``clearance_m`` from the sphere center.

    ``clearance_m`` is the tip-to-center distance (meters). Use values larger
    than the marker radius so the EE stays outside the volumetric obstacle.
    """
    start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    clearance = max(1e-4, float(clearance_m))
    delta = center - start
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        return center - np.array([0.0, 0.0, clearance], dtype=float)
    return center - (clearance / dist) * delta


def recovery_via_attempts_in_message(message: str) -> int:
    """Count ``via1_`` planning legs recorded in a plan message."""
    return len(re.findall(r"via1_", str(message or "")))


def recovery_direct_attempted(message: str) -> bool:
    return "direct:" in str(message or "") or "strategy=direct" in str(message or "")


def recovery_audit_plan_fail(
    message: str,
    *,
    recovery_enabled: bool = True,
    had_marker: bool = True,
) -> dict[str, object]:
    """Audit a failed plan message for the 'yellow / no recovery' bug class.

    When recovery is enabled and a volumetric marker was present, a PLAN_FAIL
    must show at least one ``via1_`` attempt (unless the message explicitly
    records that vias were impossible). Returns a dict with ``ok`` and
    ``reason``.
    """
    msg = str(message or "")
    vias = recovery_via_attempts_in_message(msg)
    if not recovery_enabled:
        return {"ok": True, "reason": "recovery_disabled", "via_attempts": vias}
    if not had_marker:
        return {"ok": True, "reason": "no_marker", "via_attempts": vias}
    if "recovery_skipped" in msg or "curobo_unavailable" in msg:
        return {"ok": False, "reason": "recovery_skipped", "via_attempts": vias}
    if vias < 1 and "skip_near" not in msg and "no_marker" not in msg:
        return {
            "ok": False,
            "reason": "no_via_attempt_after_direct_fail",
            "via_attempts": vias,
        }
    return {"ok": True, "reason": "vias_attempted", "via_attempts": vias}


def _select_marker(
    obstacles: Sequence[SphereObstacle],
    tip_goal_m: np.ndarray,
) -> SphereObstacle | None:
    if not obstacles:
        return None
    tip_goal = np.asarray(tip_goal_m, dtype=float).reshape(3)
    for candidate in obstacles:
        if (
            float(
                np.linalg.norm(
                    np.asarray(candidate.center_m, dtype=float).reshape(3) - tip_goal
                )
            )
            < 1e-4
        ):
            return candidate
    return obstacles[0]


def _concat_waypoints(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    wa = np.asarray(a, dtype=float)
    wb = np.asarray(b, dtype=float)
    if wa.size == 0:
        return wb
    if wb.size == 0:
        return wa
    if np.allclose(wa[-1], wb[0], atol=1e-5):
        return np.vstack([wa, wb[1:]])
    return np.vstack([wa, wb])


def _fail(
    message: str,
    *,
    backend: str = "curobo",
    dt_s: float = 0.02,
) -> PlannedTrajectory:
    return PlannedTrajectory(
        waypoints_rad=np.zeros((0, 6)),
        dt_s=dt_s,
        success=False,
        backend=backend,
        message=message,
    )


def plan_via_standoff(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    planner: CuRoboMotionPlanner,
    obstacles: list[SphereObstacle] | None = None,
    model: UrdfKinematicModel | None = None,
    standoff_clearances_m: Sequence[float] | None = None,
    surface_margin_m: float = 0.008,
    max_attempts: int = 4,
    timeout_s: float = 15.0,
    deadline_monotonic: float | None = None,
) -> PlannedTrajectory:
    """Try direct surface approach, then standoff→contact via plans until timeout.

    Recovery order
    --------------
    1. Direct plan to marker surface (capped attempts so vias get wall time).
    2. For each clearance in ``standoff_clearances_m``: plan
       ``start → standoff`` then ``standoff → surface`` and concatenate.

    After a failed direct plan, **at least one** via clearance is always
    attempted when a marker exists (even if the soft deadline already elapsed
    during the direct call). Further clearances respect the deadline.

    Never returns an unsafe NumPy lerp.
    """
    mdl = model or get_default_model()
    cfg = load_planning_config()
    clearances = list(
        standoff_clearances_m
        if standoff_clearances_m is not None
        else cfg.get("plan_recovery_standoff_clearances_m", [0.05, 0.08, 0.12])
    )
    surface_margin = float(
        cfg.get("plan_recovery_surface_margin_m", surface_margin_m)
    )
    attempts = int(cfg.get("curobo_max_attempts", max_attempts))
    # Cap direct attempts so a slow IK_FAIL cannot consume the whole timeout
    # before vias run (the GUI "yellow / motionless / no recovery" failure mode).
    direct_attempts = int(cfg.get("plan_recovery_direct_max_attempts", min(2, attempts)))
    via_attempts = int(cfg.get("plan_recovery_via_max_attempts", attempts))
    t0 = time.monotonic()
    timeout = float(timeout_s)
    deadline = (
        float(deadline_monotonic)
        if deadline_monotonic is not None
        else t0 + timeout
    )

    def _timed_out() -> bool:
        return time.monotonic() >= deadline

    obs = list(obstacles or [])
    pose_goal = forward_kinematics(q_goal_rad, model=mdl)
    tip_goal = np.asarray(pose_goal.position_m, dtype=float).reshape(3)
    tip_start = forward_kinematics(q_start_rad, model=mdl).position_m
    quat = pose_goal.quaternion_wxyz
    marker = _select_marker(obs, tip_goal)
    attempts_log: list[str] = []
    via_legs_started = 0

    # --- Strategy 1: direct surface ---
    direct = planner.plan_to_joint_goal(
        q_start_rad,
        q_goal_rad,
        model=mdl,
        obstacles=obs,
        max_attempts=max(1, direct_attempts),
    )
    attempts_log.append(f"direct:{direct.message}")
    if direct.ok:
        return PlannedTrajectory(
            waypoints_rad=direct.waypoints_rad,
            dt_s=direct.dt_s,
            success=True,
            backend=direct.backend,
            message=f"ok|strategy=direct|{direct.message}",
        )

    if marker is None:
        return _fail(
            f"recovery_exhausted_no_marker|via_attempts=0|{'|'.join(attempts_log)}",
            dt_s=planner._interpolation_dt_s,
        )

    center = np.asarray(marker.center_m, dtype=float).reshape(3)
    radius = float(marker.radius_m)
    tip_contact = tip_on_sphere_surface(
        tip_start, center, radius, margin_m=surface_margin
    )

    # --- Strategy 2: via standoff clearances ---
    for idx, clearance in enumerate(clearances):
        # Always allow the first via after a failed direct, even if direct burned
        # the soft deadline. Later clearances respect the timeout.
        if idx > 0 and _timed_out():
            attempts_log.append("timeout_before_remaining_vias")
            break
        clearance_f = float(clearance)
        min_clear = radius + surface_margin + 0.01
        if clearance_f < min_clear:
            clearance_f = min_clear
        tip_standoff = tip_standoff_on_approach(
            tip_start, center, clearance_m=clearance_f
        )
        if float(np.linalg.norm(tip_standoff - tip_contact)) < 0.015:
            attempts_log.append(f"standoff_{clearance_f:.3f}:skip_near_contact")
            continue
        if float(np.linalg.norm(tip_standoff - tip_start)) < 0.01:
            attempts_log.append(f"standoff_{clearance_f:.3f}:skip_near_start")
            continue

        via_legs_started += 1
        leg1 = planner.plan_to_pose(
            q_start_rad,
            tip_standoff,
            quat,
            max_attempts=max(1, via_attempts),
            obstacles=obs,
        )
        attempts_log.append(f"via1_{clearance_f:.3f}:{leg1.message}")
        if not leg1.ok:
            continue
        q_mid = np.asarray(leg1.waypoints_rad[-1], dtype=float).reshape(6)
        if idx > 0 and _timed_out():
            attempts_log.append("timeout_before_via2")
            break
        leg2 = planner.plan_to_pose(
            q_mid,
            tip_contact,
            quat,
            max_attempts=max(1, via_attempts),
            obstacles=obs,
        )
        attempts_log.append(f"via2_{clearance_f:.3f}:{leg2.message}")
        if not leg2.ok:
            continue
        waypoints = _concat_waypoints(leg1.waypoints_rad, leg2.waypoints_rad)
        return PlannedTrajectory(
            waypoints_rad=waypoints,
            dt_s=float(leg1.dt_s),
            success=True,
            backend="curobo",
            message=(
                f"ok|strategy=via_standoff|clearance_m={clearance_f:.3f}|"
                f"via_attempts={via_legs_started}|"
                f"elapsed_s={time.monotonic() - t0:.2f}"
            ),
        )

    elapsed = time.monotonic() - t0
    reason = "timeout" if _timed_out() and via_legs_started == 0 else (
        "timeout" if _timed_out() else "exhausted"
    )
    # Prefer a precise label when vias never started despite a marker.
    if via_legs_started == 0 and any("skip_near" in x for x in attempts_log):
        reason = "exhausted_skips"
    elif via_legs_started == 0:
        reason = "no_via_attempt"
    return _fail(
        f"recovery_{reason}|via_attempts={via_legs_started}|"
        f"elapsed_s={elapsed:.2f}|{'|'.join(attempts_log)}",
        dt_s=planner._interpolation_dt_s,
    )


def plan_collision_free_with_recovery(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    prefer_curobo: bool = True,
    model: UrdfKinematicModel | None = None,
    obstacles: list[SphereObstacle] | None = None,
    planner: CuRoboMotionPlanner | None = None,
    enable_recovery: bool | None = None,
    timeout_s: float | None = None,
) -> PlannedTrajectory:
    """Plan with optional standoff-via recovery (see ``plan_via_standoff``).

    When recovery is disabled or cuRobo is unavailable, delegates to
    ``plan_collision_free`` (fail-closed NumPy policy unchanged).
    """
    from residual_adaptive_ik.planning.curobo_planner import plan_collision_free

    cfg = load_planning_config()
    recover = (
        bool(cfg.get("plan_recovery_enabled", True))
        if enable_recovery is None
        else bool(enable_recovery)
    )
    timeout = float(
        cfg.get("plan_recovery_timeout_s", 15.0) if timeout_s is None else timeout_s
    )

    if not (recover and prefer_curobo and curobo_available()):
        traj = plan_collision_free(
            q_start_rad,
            q_goal_rad,
            prefer_curobo=prefer_curobo,
            model=model,
            obstacles=obstacles,
            planner=planner,
        )
        if recover and prefer_curobo and not curobo_available() and not traj.ok:
            return PlannedTrajectory(
                waypoints_rad=traj.waypoints_rad,
                dt_s=traj.dt_s,
                success=False,
                backend=traj.backend,
                message=f"recovery_skipped|curobo_unavailable|{traj.message}",
            )
        return traj

    pl = planner or CuRoboMotionPlanner(omit_tip_links=False)
    return plan_via_standoff(
        q_start_rad,
        q_goal_rad,
        planner=pl,
        obstacles=obstacles,
        model=model,
        timeout_s=timeout,
    )
