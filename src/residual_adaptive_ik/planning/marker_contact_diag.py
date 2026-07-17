# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Headless diagnostics: volumetric IK marker vs EE / arm collision spheres.

Why this exists
---------------
GUI review of marker–EE contact is slow. This module classifies whether the
12 mm target sphere intersects **tip-zone** robot spheres (expected near
contact) or **side / proximal** spheres (undesired — marker through EE body).

Units: meters, radians. See ``scripts/host/diagnose_marker_ee_contact.sh`` and
``docs/phase2_geometry.md``.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

# Keep in sync with ``isaac_sim/target_marker.TARGET_MARKER_RADIUS_M`` (meters).
DEFAULT_MARKER_RADIUS_M = 0.012


@dataclass(frozen=True)
class SphereHit:
    """One robot collision sphere intersecting the marker volume."""

    sphere_index: int
    center_m: tuple[float, float, float]
    radius_m: float
    dist_to_marker_m: float
    dist_to_tip_m: float
    lateral_m: float
    axial_m: float
    kind: str  # "tip" | "side"


@dataclass
class WaypointContactReport:
    """Marker contact summary at one joint configuration."""

    waypoint_index: int
    tip_m: tuple[float, float, float]
    tip_to_marker_m: float
    tip_inside_marker: bool
    n_tip_hits: int = 0
    n_side_hits: int = 0
    hits: list[SphereHit] = field(default_factory=list)

    @property
    def has_side_contact(self) -> bool:
        return self.n_side_hits > 0


@dataclass
class TrialContactReport:
    """Per-trial planning + contact scan."""

    trial_index: int
    plan_success: bool
    plan_backend: str
    plan_message: str
    marker_center_m: tuple[float, float, float]
    marker_radius_m: float
    n_waypoints: int
    n_waypoints_with_side: int
    n_waypoints_with_tip: int
    max_side_hits_on_waypoint: int
    first_side_waypoint: int | None
    waypoints: list[WaypointContactReport] = field(default_factory=list)

    @property
    def has_side_contact(self) -> bool:
        return self.n_waypoints_with_side > 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def classify_sphere_vs_marker(
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    tip_m: np.ndarray,
    marker_center_m: np.ndarray,
    marker_radius_m: float,
    *,
    tip_zone_m: float = 0.028,
    lateral_side_m: float = 0.010,
    approach_m: np.ndarray | None = None,
) -> SphereHit | None:
    """Return a hit if the robot sphere intersects the marker volume.

    Classification
    --------------
    * **tip** — sphere center within ``tip_zone_m`` of the FK tip, and lateral
      offset from the approach axis ≤ ``lateral_side_m``.
    * **side** — any other intersecting sphere (proximal EE / lateral flange).

    Approach defaults to ``marker_center - tip`` (meters).
    """
    c = np.asarray(sphere_center_m, dtype=float).reshape(3)
    tip = np.asarray(tip_m, dtype=float).reshape(3)
    marker = np.asarray(marker_center_m, dtype=float).reshape(3)
    r_s = float(sphere_radius_m)
    r_m = float(marker_radius_m)
    dist_marker = float(np.linalg.norm(c - marker))
    if dist_marker > (r_s + r_m + 1e-9):
        return None

    if approach_m is None:
        approach = marker - tip
    else:
        approach = np.asarray(approach_m, dtype=float).reshape(3)
    an = float(np.linalg.norm(approach))
    if an < 1e-9:
        approach_u = np.array([0.0, 0.0, 1.0])
    else:
        approach_u = approach / an

    v = c - tip
    axial = float(np.dot(v, approach_u))
    lateral = float(np.linalg.norm(v - axial * approach_u))
    dist_tip = float(np.linalg.norm(v))

    if dist_tip <= float(tip_zone_m) and lateral <= float(lateral_side_m):
        kind = "tip"
    else:
        kind = "side"

    return SphereHit(
        sphere_index=-1,
        center_m=(float(c[0]), float(c[1]), float(c[2])),
        radius_m=r_s,
        dist_to_marker_m=dist_marker,
        dist_to_tip_m=dist_tip,
        lateral_m=lateral,
        axial_m=axial,
        kind=kind,
    )


def settle_has_side_sphere_hits(
    robot_spheres_xyzr: np.ndarray,
    tip_m: np.ndarray,
    marker_center_m: np.ndarray,
    *,
    marker_radius_m: float = DEFAULT_MARKER_RADIUS_M,
    tip_zone_m: float = 0.028,
    lateral_side_m: float = 0.010,
    approach_m: np.ndarray | None = None,
) -> tuple[bool, WaypointContactReport]:
    """Return whether a settle pose has non-tip spheres intersecting the marker.

    Why this exists
    ---------------
    Point-tip ``classify_tip_contact`` can still flash green while flange /
    barrel collision spheres clip the marker (tip-omit turns those spheres off
    for MotionGen, but the volumetric EE body is still wrong). Tip-zone hits
    alone are expected at contact; any **side** hit must fail the settle gate
    (``PLAN_FAIL(invalid_side)`` / ``MARKER_EE_SIDE_SPHERE``).

    Pure NumPy — no Kit. Units: meters.
    """
    report = analyze_waypoint_spheres(
        robot_spheres_xyzr,
        tip_m,
        marker_center_m,
        marker_radius_m=marker_radius_m,
        waypoint_index=0,
        tip_zone_m=tip_zone_m,
        lateral_side_m=lateral_side_m,
        approach_m=approach_m,
    )
    return report.has_side_contact, report


def analyze_waypoint_spheres(
    robot_spheres_xyzr: np.ndarray,
    tip_m: np.ndarray,
    marker_center_m: np.ndarray,
    *,
    marker_radius_m: float = DEFAULT_MARKER_RADIUS_M,
    waypoint_index: int = 0,
    tip_zone_m: float = 0.028,
    lateral_side_m: float = 0.010,
    approach_m: np.ndarray | None = None,
) -> WaypointContactReport:
    """Classify all robot spheres at one configuration (shape ``(N, 4)``)."""
    sph = np.asarray(robot_spheres_xyzr, dtype=float).reshape(-1, 4)
    tip = np.asarray(tip_m, dtype=float).reshape(3)
    marker = np.asarray(marker_center_m, dtype=float).reshape(3)
    tip_to_marker = float(np.linalg.norm(tip - marker))
    hits: list[SphereHit] = []
    for i, row in enumerate(sph):
        hit = classify_sphere_vs_marker(
            row[:3],
            float(row[3]),
            tip,
            marker,
            marker_radius_m,
            tip_zone_m=tip_zone_m,
            lateral_side_m=lateral_side_m,
            approach_m=approach_m,
        )
        if hit is None:
            continue
        hits.append(
            SphereHit(
                sphere_index=i,
                center_m=hit.center_m,
                radius_m=hit.radius_m,
                dist_to_marker_m=hit.dist_to_marker_m,
                dist_to_tip_m=hit.dist_to_tip_m,
                lateral_m=hit.lateral_m,
                axial_m=hit.axial_m,
                kind=hit.kind,
            )
        )
    n_tip = sum(1 for h in hits if h.kind == "tip")
    n_side = sum(1 for h in hits if h.kind == "side")
    return WaypointContactReport(
        waypoint_index=int(waypoint_index),
        tip_m=(float(tip[0]), float(tip[1]), float(tip[2])),
        tip_to_marker_m=tip_to_marker,
        tip_inside_marker=tip_to_marker <= marker_radius_m + 1e-9,
        n_tip_hits=n_tip,
        n_side_hits=n_side,
        hits=hits,
    )


def summarize_trial_waypoints(
    waypoint_reports: list[WaypointContactReport],
    *,
    trial_index: int,
    plan_success: bool,
    plan_backend: str,
    plan_message: str,
    marker_center_m: np.ndarray,
    marker_radius_m: float = DEFAULT_MARKER_RADIUS_M,
) -> TrialContactReport:
    """Aggregate waypoint reports into a trial summary."""
    marker = np.asarray(marker_center_m, dtype=float).reshape(3)
    side_idx = [w.waypoint_index for w in waypoint_reports if w.has_side_contact]
    tip_idx = [w.waypoint_index for w in waypoint_reports if w.n_tip_hits > 0]
    return TrialContactReport(
        trial_index=int(trial_index),
        plan_success=bool(plan_success),
        plan_backend=str(plan_backend),
        plan_message=str(plan_message),
        marker_center_m=(float(marker[0]), float(marker[1]), float(marker[2])),
        marker_radius_m=float(marker_radius_m),
        n_waypoints=len(waypoint_reports),
        n_waypoints_with_side=len(side_idx),
        n_waypoints_with_tip=len(tip_idx),
        max_side_hits_on_waypoint=max(
            (w.n_side_hits for w in waypoint_reports), default=0
        ),
        first_side_waypoint=side_idx[0] if side_idx else None,
        waypoints=waypoint_reports,
    )
