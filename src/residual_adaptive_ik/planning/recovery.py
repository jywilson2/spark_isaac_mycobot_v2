# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Motion-planning recovery: standoff vias with timeout + optional partial exec.

Why
---
Direct tip→marker-surface plans often ``IK_FAIL`` when the start pose is awkward
or the EE is already near the volumetric target. Industrial practice splits the
motion into **via** segments: plan to a standoff pose outside the obstacle, then
a short final approach to the contact surface.

Behaviour (Phase 2 viz / smoke)
-------------------------------
* Keep retrying strategies until ``plan_recovery_timeout_s`` elapses — do **not**
  declare PLAN_FAIL after a single quick pass over clearances.
* After a failed attempt, via standoffs are ordered **nearest → farthest** from
  the current tip (progressive distance). Candidates closer than
  ``plan_recovery_min_standoff_travel_m`` are skipped so the EE does not “retry”
  an almost-identical pose; each subsequent candidate is farther out.
* ``INVALID_START_STATE_WORLD_COLLISION`` escapes toward home a few times, then
  **falls through to via standoffs** once the escape weight saturates. This
  avoids burning the full timeout on an unresolvable start-in-obstacle loop.
* When ``execute_waypoints`` is provided and via1 succeeds but via2 fails,
  **execute via1** (move the EE), then continue planning from the new pose.
* The IK target marker must not jump to yellow until this budget is exhausted.

This is classical motion planning — not residual learning. See
``docs/phase2_geometry.md`` and ``configs/planning/collision.yaml``.

Units: meters, radians, seconds.
"""
from __future__ import annotations

import math
import re
import time
from typing import Callable, Sequence

import numpy as np

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel, get_default_model
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    PlannedTrajectory,
    curobo_available,
    load_planning_config,
    tip_on_sphere_surface,
)

# Called as ``execute_waypoints(waypoints_rad, dt_s)`` to move the arm mid-recovery.
ExecuteWaypointsFn = Callable[[np.ndarray, float], None]


def tip_standoff_on_approach(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    *,
    clearance_m: float,
    yaw_offset_rad: float = 0.0,
) -> np.ndarray:
    """Tip pose at ``clearance_m`` from the sphere center (meters).

    ``yaw_offset_rad`` rotates the approach direction about +Z through the
    sphere center so recovery can try lateral standoffs, not only the pure
    tip→center ray.
    """
    start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    clearance = max(1e-4, float(clearance_m))
    delta = center - start
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        direction = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        direction = delta / dist
    yaw = float(yaw_offset_rad)
    if abs(yaw) > 1e-9:
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z = direction
        direction = np.array([c * x - s * y, s * x + c * y, z], dtype=float)
        n = float(np.linalg.norm(direction))
        if n < 1e-9:
            direction = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            direction = direction / n
    return center - clearance * direction


def ordered_standoff_candidates(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    tip_contact_m: np.ndarray,
    *,
    clearances_m: Sequence[float],
    yaws_rad: Sequence[float],
    radius_m: float,
    surface_margin_m: float,
    min_travel_m: float,
    skip_near_contact_m: float = 0.015,
) -> list[tuple[float, float, np.ndarray, float, str]]:
    """Build via-standoff candidates, nearest to the current tip first.

    Progressive recovery: try a nearby standoff first, then increasingly
    distant waypoints on later retries. Candidates with tip travel
    ``< min_travel_m`` are omitted (``skip_near_start``) so we do not plan to
    an almost-identical EE pose.

    Returns
    -------
    list of ``(clearance_m, yaw_rad, tip_standoff_m, travel_m, skip_reason)``
    where ``skip_reason`` is empty for usable candidates, else a short tag.
    Usable rows are sorted by ``travel_m`` **ascending** (near → far).
    """
    tip_start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    tip_contact = np.asarray(tip_contact_m, dtype=float).reshape(3)
    min_clear = float(radius_m) + float(surface_margin_m) + 0.01
    min_travel = max(0.0, float(min_travel_m))
    usable: list[tuple[float, float, np.ndarray, float, str]] = []
    skipped: list[tuple[float, float, np.ndarray, float, str]] = []
    for clearance in clearances_m:
        clearance_f = max(float(clearance), min_clear)
        for yaw in yaws_rad:
            tip_standoff = tip_standoff_on_approach(
                tip_start,
                center,
                clearance_m=clearance_f,
                yaw_offset_rad=float(yaw),
            )
            travel = float(np.linalg.norm(tip_standoff - tip_start))
            if float(np.linalg.norm(tip_standoff - tip_contact)) < skip_near_contact_m:
                skipped.append(
                    (clearance_f, float(yaw), tip_standoff, travel, "skip_near_contact")
                )
                continue
            if travel < min_travel:
                skipped.append(
                    (clearance_f, float(yaw), tip_standoff, travel, "skip_near_start")
                )
                continue
            usable.append((clearance_f, float(yaw), tip_standoff, travel, ""))
    usable.sort(key=lambda row: row[3])  # nearest first, then farther
    return usable + skipped


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


def _clamp_joints(q_rad: np.ndarray) -> np.ndarray:
    """Clamp to URDF joint limits with a small interior margin (radians)."""
    lo, hi = load_joint_limits_rad()
    margin = 1e-3
    return np.clip(
        np.asarray(q_rad, dtype=float).reshape(6),
        lo + margin,
        hi - margin,
    )


def _blend_toward_home(q_rad: np.ndarray, *, weight: float = 0.35) -> np.ndarray:
    """Pull joints toward home and clamp (helps INVALID_START_* escapes)."""
    from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad

    q = np.asarray(q_rad, dtype=float).reshape(6)
    home = load_home_joint_positions_rad().reshape(6)
    w = float(np.clip(weight, 0.0, 1.0))
    return _clamp_joints((1.0 - w) * q + w * home)


def _is_invalid_start(message: str) -> bool:
    m = str(message or "").upper()
    return "INVALID_START" in m


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


def _ok(
    waypoints: np.ndarray,
    *,
    dt_s: float,
    message: str,
    backend: str = "curobo",
) -> PlannedTrajectory:
    wp = np.asarray(waypoints, dtype=float)
    if wp.ndim == 1:
        wp = wp.reshape(1, -1)
    if wp.size == 0:
        wp = np.zeros((1, 6))
    return PlannedTrajectory(
        waypoints_rad=wp,
        dt_s=dt_s,
        success=True,
        backend=backend,
        message=message,
    )


def plan_via_standoff(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    planner: CuRoboMotionPlanner,
    contact_planner: CuRoboMotionPlanner | None = None,
    obstacles: list[SphereObstacle] | None = None,
    model: UrdfKinematicModel | None = None,
    standoff_clearances_m: Sequence[float] | None = None,
    surface_margin_m: float = 0.008,
    max_attempts: int = 4,
    timeout_s: float = 15.0,
    deadline_monotonic: float | None = None,
    execute_waypoints: ExecuteWaypointsFn | None = None,
) -> PlannedTrajectory:
    """Retry direct + standoff→contact until timeout; optionally move on via1.

    Recovery order (repeated until deadline)
    ---------------------------------------
    1. Direct plan to marker surface (contact planner / omit tip spheres).
    2. Standoff candidates (clearance × lateral yaw), sorted by tip travel
       **ascending** (near → far); skip near-start / near-contact vias.
    3. If via1 succeeds and via2 fails and ``execute_waypoints`` is set,
       execute via1 (EE moves), update the start pose, and keep trying.
    4. Only after the wall-clock budget expires return a failed trajectory.

    Never returns an unsafe NumPy lerp.
    """
    mdl = model or get_default_model()
    cfg = load_planning_config()
    clearances = list(
        standoff_clearances_m
        if standoff_clearances_m is not None
        else cfg.get(
            "plan_recovery_standoff_clearances_m",
            [0.04, 0.06, 0.08, 0.10, 0.14, 0.18],
        )
    )
    yaws = list(
        cfg.get("plan_recovery_lateral_yaw_rad", [0.0, 0.4, -0.4, 0.8, -0.8])
    )
    surface_margin = float(
        cfg.get("plan_recovery_surface_margin_m", surface_margin_m)
    )
    # Floor only: skip tiny no-ops; progressive order tries near then far.
    min_standoff_travel_m = float(
        cfg.get("plan_recovery_min_standoff_travel_m", 0.01)
    )
    attempts = int(cfg.get("curobo_max_attempts", max_attempts))
    direct_attempts = int(
        cfg.get("plan_recovery_direct_max_attempts", min(2, attempts))
    )
    via_attempts = int(cfg.get("plan_recovery_via_max_attempts", attempts))
    contact = contact_planner or planner
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
    quat = pose_goal.quaternion_wxyz
    marker = _select_marker(obs, tip_goal)
    attempts_log: list[str] = []
    via_legs_started = 0
    partial_execs = 0
    q_cur = _clamp_joints(q_start_rad)
    tip_start = forward_kinematics(q_cur, model=mdl).position_m
    contact_margin = (
        0.0 if bool(getattr(contact, "_omit_tip_links", False)) else surface_margin
    )
    last_dt = float(getattr(planner, "_interpolation_dt_s", 0.02))
    escape_weight = 0.4

    if marker is None:
        direct = contact.plan_to_joint_goal(
            q_cur,
            q_goal_rad,
            model=mdl,
            obstacles=obs,
            max_attempts=max(1, direct_attempts),
        )
        attempts_log.append(f"direct:{direct.message}")
        if direct.ok:
            return _ok(
                direct.waypoints_rad,
                dt_s=direct.dt_s,
                message=f"ok|strategy=direct|{direct.message}",
                backend=direct.backend,
            )
        return _fail(
            f"recovery_exhausted_no_marker|via_attempts=0|{'|'.join(attempts_log)}",
            dt_s=last_dt,
        )

    center = np.asarray(marker.center_m, dtype=float).reshape(3)
    radius = float(marker.radius_m)

    # Always run at least one recovery pass (even if the soft deadline already
    # elapsed during setup) so logs show via1_ attempts for the audit contract.
    first_pass = True
    while first_pass or not _timed_out():
        allow_overtime = first_pass
        first_pass = False
        tip_contact = tip_on_sphere_surface(
            tip_start, center, radius, margin_m=contact_margin
        )

        direct = contact.plan_to_joint_goal(
            q_cur,
            q_goal_rad,
            model=mdl,
            obstacles=obs,
            max_attempts=max(1, direct_attempts),
        )
        attempts_log.append(f"direct:{direct.message}")
        last_dt = float(direct.dt_s)
        if direct.ok:
            msg = (
                f"ok|strategy=direct|partial_execs={partial_execs}|"
                f"elapsed_s={time.monotonic() - t0:.2f}|{direct.message}"
            )
            if execute_waypoints is not None:
                execute_waypoints(direct.waypoints_rad, float(direct.dt_s))
                q_hold = _clamp_joints(direct.waypoints_rad[-1])
                return _ok(
                    q_hold.reshape(1, 6),
                    dt_s=direct.dt_s,
                    message=msg + "|already_executed",
                    backend=direct.backend,
                )
            return _ok(
                direct.waypoints_rad,
                dt_s=direct.dt_s,
                message=msg,
                backend=direct.backend,
            )

        # Invalid start (joint limits / world collision): blend toward home
        # up to a few times to escape trivial self-collisions. Once the weight
        # saturates the escape is a no-op (home itself may be in world
        # collision with the marker), so fall through to standoff vias instead
        # of burning the entire timeout on identical direct rejects.
        if _is_invalid_start(direct.message):
            if escape_weight < 0.99:
                q_esc = _blend_toward_home(q_cur, weight=escape_weight)
                escape_weight = min(0.99, escape_weight + 0.15)
                attempts_log.append(
                    f"invalid_start_escape_toward_home_w{escape_weight:.2f}"
                )
                if execute_waypoints is not None:
                    execute_waypoints(
                        np.vstack([q_cur.reshape(1, 6), q_esc.reshape(1, 6)]),
                        max(last_dt, 0.02),
                    )
                q_cur = q_esc
                tip_start = forward_kinematics(q_cur, model=mdl).position_m
                time.sleep(0.02)
                continue
            # Saturated: home escape exhausted — fall through to via standoffs
            # with a fresh escape budget for the via loop.
            escape_weight = 0.40
            attempts_log.append("invalid_start_escape_saturated_trying_vias")

        progressed = False
        candidates = ordered_standoff_candidates(
            tip_start,
            center,
            tip_contact,
            clearances_m=clearances,
            yaws_rad=yaws,
            radius_m=radius,
            surface_margin_m=surface_margin,
            min_travel_m=min_standoff_travel_m,
        )
        for clearance_f, yaw, tip_standoff, travel_m, skip_reason in candidates:
            if _timed_out() and not allow_overtime:
                break
            if skip_reason:
                attempts_log.append(
                    f"standoff_{clearance_f:.3f}_y{yaw:.2f}:{skip_reason}"
                    f"|travel_m={travel_m:.3f}"
                )
                continue

            via_legs_started += 1
            leg1 = planner.plan_to_pose(
                q_cur,
                tip_standoff,
                quat,
                max_attempts=max(1, via_attempts),
                obstacles=obs,
            )
            tag = f"{clearance_f:.3f}_y{yaw:.2f}"
            attempts_log.append(
                f"via1_{tag}:travel_m={travel_m:.3f}|{leg1.message}"
            )
            last_dt = float(leg1.dt_s)
            if not leg1.ok:
                continue

            q_mid = _clamp_joints(leg1.waypoints_rad[-1])
            tip_mid = forward_kinematics(q_mid, model=mdl).position_m
            # If the standoff already places the tip on/inside the sphere
            # surface, treat as success (no via2 required).
            if float(np.linalg.norm(tip_mid - center)) <= radius + 0.003:
                if execute_waypoints is not None:
                    execute_waypoints(leg1.waypoints_rad, float(leg1.dt_s))
                    return _ok(
                        q_mid.reshape(1, 6),
                        dt_s=float(leg1.dt_s),
                        message=(
                            f"ok|strategy=via_standoff_contact|"
                            f"clearance_m={clearance_f:.3f}|yaw_rad={yaw:.2f}|"
                            f"via_attempts={via_legs_started}|"
                            f"partial_execs={partial_execs}|"
                            f"elapsed_s={time.monotonic() - t0:.2f}|already_executed"
                        ),
                    )
                return _ok(
                    leg1.waypoints_rad,
                    dt_s=float(leg1.dt_s),
                    message=(
                        f"ok|strategy=via_standoff_contact|"
                        f"clearance_m={clearance_f:.3f}|yaw_rad={yaw:.2f}|"
                        f"via_attempts={via_legs_started}|"
                        f"elapsed_s={time.monotonic() - t0:.2f}"
                    ),
                )
            tip_contact = tip_on_sphere_surface(
                tip_mid, center, radius, margin_m=contact_margin
            )
            if _timed_out() and not allow_overtime:
                if execute_waypoints is not None:
                    execute_waypoints(leg1.waypoints_rad, float(leg1.dt_s))
                    partial_execs += 1
                    q_cur = q_mid
                    tip_start = tip_mid
                    progressed = True
                attempts_log.append("timeout_before_via2")
                break

            leg2 = None
            quat_mid = forward_kinematics(q_mid, model=mdl).quaternion_wxyz
            tip_margins = [contact_margin, 0.003, 0.008, 0.015]
            for cm in tip_margins:
                tip_try = tip_on_sphere_surface(
                    tip_mid, center, radius, margin_m=float(cm)
                )
                for quat_try, quat_tag in (
                    (quat_mid, "mid_q"),
                    (quat, "goal_q"),
                ):
                    if _timed_out() and not allow_overtime:
                        break
                    leg2 = contact.plan_to_pose(
                        q_mid,
                        tip_try,
                        quat_try,
                        max_attempts=max(1, via_attempts),
                        obstacles=obs,
                    )
                    attempts_log.append(
                        f"via2_{tag}_m{cm:.3f}_{quat_tag}:{leg2.message}"
                    )
                    last_dt = float(leg2.dt_s)
                    if leg2.ok:
                        break
                if leg2 is not None and leg2.ok:
                    break
                # Classical IK seed → joint-space MotionGen to surface tip.
                try:
                    from residual_adaptive_ik.kinematics.fk import Pose
                    from residual_adaptive_ik.kinematics.numerical_ik import (
                        DampedLeastSquaresIK,
                    )

                    ik = DampedLeastSquaresIK(model=mdl)
                    pose_try = Pose(
                        position_m=tip_try, quaternion_wxyz=quat_mid
                    )
                    ik_res = ik.solve(pose_try, seed_q=q_mid)
                    if bool(getattr(ik_res, "success", False)):
                        q_ik = _clamp_joints(ik_res.q)
                        leg2 = contact.plan_to_joint_goal(
                            q_mid,
                            q_ik,
                            model=mdl,
                            obstacles=obs,
                            max_attempts=max(1, via_attempts),
                        )
                        attempts_log.append(
                            f"via2_{tag}_m{cm:.3f}_ik:{leg2.message}"
                        )
                        last_dt = float(leg2.dt_s)
                        if leg2.ok:
                            break
                except Exception as exc:  # noqa: BLE001
                    attempts_log.append(f"via2_{tag}_ik_err:{exc}")
            if leg2 is not None and leg2.ok:
                if execute_waypoints is not None:
                    execute_waypoints(leg1.waypoints_rad, float(leg1.dt_s))
                    execute_waypoints(leg2.waypoints_rad, float(leg2.dt_s))
                    q_hold = _clamp_joints(leg2.waypoints_rad[-1])
                    return _ok(
                        q_hold.reshape(1, 6),
                        dt_s=float(leg1.dt_s),
                        message=(
                            f"ok|strategy=via_standoff|clearance_m={clearance_f:.3f}|"
                            f"yaw_rad={yaw:.2f}|via_attempts={via_legs_started}|"
                            f"partial_execs={partial_execs}|"
                            f"elapsed_s={time.monotonic() - t0:.2f}|already_executed"
                        ),
                    )
                waypoints = _concat_waypoints(
                    leg1.waypoints_rad, leg2.waypoints_rad
                )
                return _ok(
                    waypoints,
                    dt_s=float(leg1.dt_s),
                    message=(
                        f"ok|strategy=via_standoff|clearance_m={clearance_f:.3f}|"
                        f"yaw_rad={yaw:.2f}|via_attempts={via_legs_started}|"
                        f"elapsed_s={time.monotonic() - t0:.2f}"
                    ),
                )

            if execute_waypoints is not None:
                execute_waypoints(leg1.waypoints_rad, float(leg1.dt_s))
                partial_execs += 1
                q_cur = q_mid
                tip_start = tip_mid
                progressed = True
                attempts_log.append(f"partial_via1_{tag}:executed")
                break

        if _timed_out() and not allow_overtime:
            break
        if not progressed:
            # If via attempts all hit INVALID_START, the start pose is in
            # collision. Blend toward home (same logic as the direct-plan
            # escape) so the next iteration retries from a safer pose.
            recent = "|".join(attempts_log[-20:]).upper()
            if "INVALID_START" in recent and escape_weight < 0.99:
                q_esc = _blend_toward_home(q_cur, weight=escape_weight)
                escape_weight = min(0.99, escape_weight + 0.15)
                attempts_log.append(
                    f"via_invalid_start_escape_w{escape_weight:.2f}"
                )
                if execute_waypoints is not None:
                    execute_waypoints(
                        np.vstack([q_cur.reshape(1, 6), q_esc.reshape(1, 6)]),
                        max(last_dt, 0.02),
                    )
                q_cur = q_esc
                tip_start = forward_kinematics(q_cur, model=mdl).position_m
            time.sleep(0.05)

    elapsed = time.monotonic() - t0
    log_s = "|".join(attempts_log)
    if len(log_s) > 2500:
        log_s = log_s[:1200] + "|…|" + log_s[-1200:]
    return _fail(
        f"recovery_timeout|via_attempts={via_legs_started}|"
        f"partial_execs={partial_execs}|elapsed_s={elapsed:.2f}|{log_s}",
        dt_s=last_dt,
    )


def plan_collision_free_with_recovery(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    prefer_curobo: bool = True,
    model: UrdfKinematicModel | None = None,
    obstacles: list[SphereObstacle] | None = None,
    planner: CuRoboMotionPlanner | None = None,
    contact_planner: CuRoboMotionPlanner | None = None,
    enable_recovery: bool | None = None,
    timeout_s: float | None = None,
    execute_waypoints: ExecuteWaypointsFn | None = None,
) -> PlannedTrajectory:
    """Plan with optional standoff-via recovery (see ``plan_via_standoff``).

    ``execute_waypoints`` — when set (Isaac viz), successful via1 legs are
    executed immediately so the EE moves during the recovery budget.
    """
    from residual_adaptive_ik.planning.curobo_planner import plan_collision_free

    cfg = load_planning_config()
    recover = (
        bool(cfg.get("plan_recovery_enabled", True))
        if enable_recovery is None
        else bool(enable_recovery)
    )
    timeout = float(
        cfg.get("plan_recovery_timeout_s", 90.0) if timeout_s is None else timeout_s
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
    if contact_planner is not None:
        contact_pl = contact_planner
    elif getattr(pl, "_omit_tip_links", False):
        contact_pl = pl
    else:
        cached = getattr(pl, "_contact_omit_tip_planner", None)
        if cached is None:
            cached = CuRoboMotionPlanner(
                omit_tip_links=True,
                interpolation_dt_s=float(getattr(pl, "_interpolation_dt_s", 0.02)),
                urdf_path=getattr(pl, "_urdf_path", None),
            )
            setattr(pl, "_contact_omit_tip_planner", cached)
        contact_pl = cached
    return plan_via_standoff(
        q_start_rad,
        q_goal_rad,
        planner=pl,
        contact_planner=contact_pl,
        obstacles=obstacles,
        model=model,
        timeout_s=timeout,
        execute_waypoints=execute_waypoints,
    )
