# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 geometry: approximate link capsules + sphere obstacles (meters).

Why this exists
---------------
Classical DLS IK (Phase 1) returns a goal ``q`` only. It cannot answer
"does any link intersect the target sphere while moving?" — that needs an
explicit geometry layer. See ``spec.md`` Phase 2.

Design choice (CI vs NVIDIA)
----------------------------
* **CI / container (default):** pure NumPy capsule–sphere tests derived from
  URDF FK frames. No GPU, no MoveIt — tutorial-friendly and deterministic.
* **Host Isaac Sim (optional later):** PhysX contact queries and/or
  [cuRobo](https://curobo.org/) (Apache-2.0) for GPU collision-free
  trajectories. Those remain *wrappers around* classical ``q_ik`` + residual;
  they do not replace residual IK.

Units: meters and radians unless noted.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel, get_default_model


@dataclass(frozen=True)
class SphereObstacle:
    """Axis-aligned sphere obstacle in the robot base frame (meters)."""

    center_m: np.ndarray
    radius_m: float
    name: str = "obstacle"

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "center_m", np.asarray(self.center_m, dtype=float).reshape(3)
        )
        if float(self.radius_m) <= 0.0:
            raise ValueError("radius_m must be positive")


@dataclass(frozen=True)
class Capsule:
    """Finite capsule (line segment + radius) in meters."""

    a_m: np.ndarray
    b_m: np.ndarray
    radius_m: float
    name: str = "link"

    def __post_init__(self) -> None:
        object.__setattr__(self, "a_m", np.asarray(self.a_m, dtype=float).reshape(3))
        object.__setattr__(self, "b_m", np.asarray(self.b_m, dtype=float).reshape(3))


def point_segment_distance_m(
    point_m: np.ndarray, a_m: np.ndarray, b_m: np.ndarray
) -> float:
    """Shortest distance (meters) from ``point_m`` to segment ``a_m``–``b_m``."""
    p = np.asarray(point_m, dtype=float).reshape(3)
    a = np.asarray(a_m, dtype=float).reshape(3)
    b = np.asarray(b_m, dtype=float).reshape(3)
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-18:
        return float(np.linalg.norm(p - a))
    t = float(np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0))
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def capsule_sphere_collide(capsule: Capsule, sphere: SphereObstacle) -> bool:
    """True when capsule and sphere volumes intersect (meters)."""
    dist = point_segment_distance_m(sphere.center_m, capsule.a_m, capsule.b_m)
    return dist <= (capsule.radius_m + sphere.radius_m + 1e-12)


def link_capsules_from_q(
    q: np.ndarray,
    *,
    model: UrdfKinematicModel | None = None,
    link_radius_m: float = 0.025,
) -> list[Capsule]:
    """Build approximate capsules along the serial chain for configuration ``q``.

    Uses successive revolute joint origins plus the EE tip as segment endpoints.
    This is intentionally coarse (tutorial / CI) — not a mesh-accurate FCL model.
    """
    mdl = model or get_default_model()
    q_arr = np.asarray(q, dtype=float).reshape(-1)
    T_ee, frames = mdl.forward_transforms(q_arr)
    points = [p for p, _z in frames]
    points.append(T_ee[:3, 3].copy())
    capsules: list[Capsule] = []
    for i in range(len(points) - 1):
        capsules.append(
            Capsule(
                a_m=points[i],
                b_m=points[i + 1],
                radius_m=float(link_radius_m),
                name=f"seg_{i}",
            )
        )
    return capsules


@dataclass(frozen=True)
class CollisionReport:
    """Outcome of a configuration or path collision query."""

    collides: bool
    reasons: tuple[str, ...]


def check_config_collision(
    q: np.ndarray,
    obstacles: list[SphereObstacle],
    *,
    model: UrdfKinematicModel | None = None,
    link_radius_m: float = 0.025,
    ignore_tip_segment: bool = False,
) -> CollisionReport:
    """Return whether any link capsule intersects any sphere obstacle.

    Parameters
    ----------
    ignore_tip_segment:
        If True, skip the last **two** capsules (wrist→flange and prior wrist
        segment). The goal marker is a small sphere the tip is allowed to enter;
        coarse capsules otherwise flag almost every successful reach as a
        collision at the final samples. Proximal links (base→elbow) still count.
    """
    if not obstacles:
        return CollisionReport(collides=False, reasons=())
    capsules = link_capsules_from_q(q, model=model, link_radius_m=link_radius_m)
    if ignore_tip_segment and len(capsules) >= 2:
        capsules = capsules[:-2]
    elif ignore_tip_segment and capsules:
        capsules = capsules[:-1]
    reasons: list[str] = []
    for cap in capsules:
        for obs in obstacles:
            if capsule_sphere_collide(cap, obs):
                reasons.append(f"{cap.name}_vs_{obs.name}")
    return CollisionReport(collides=len(reasons) > 0, reasons=tuple(reasons))


def proximal_arm_contacts_target(
    q: np.ndarray,
    target_center_m: np.ndarray,
    *,
    target_radius_m: float,
    model: UrdfKinematicModel | None = None,
    link_radius_m: float = 0.018,
    n_ee_segments_ignored: int = 3,
    name: str = "ik_target",
) -> CollisionReport:
    """True when a **proximal** (non-EE) link capsule intersects the target.

    Success requires contact with the target **only** at the EE tip-face pad
    (spec.md Phase 2 tip-face / EE-only contact). Base / upper-arm / elbow
    capsules intersecting the marker volume is a hard failure.

    Why ignore the last ``n_ee_segments_ignored`` capsules (default 3)
    -----------------------------------------------------------------
    Coarse serial capsules near the wrist/flange (``seg_{n-3..}``) routinely
    overlap a 12 mm tip-contact sphere even on a *valid* pad approach — the
    18–25 mm tutorial radii are intentionally fat. Ignoring the distal EE
    chain keeps the settle monitor focused on genuine arm-body immersions
    (paths that sweep the forearm/upper arm through the marker).
    """
    obstacle = SphereObstacle(
        center_m=np.asarray(target_center_m, dtype=float).reshape(3),
        radius_m=float(target_radius_m),
        name=str(name),
    )
    capsules = link_capsules_from_q(
        q, model=model, link_radius_m=float(link_radius_m)
    )
    n_ignore = max(0, int(n_ee_segments_ignored))
    if n_ignore > 0 and len(capsules) > n_ignore:
        capsules = capsules[:-n_ignore]
    elif n_ignore > 0:
        capsules = []
    reasons: list[str] = []
    for cap in capsules:
        if capsule_sphere_collide(cap, obstacle):
            reasons.append(f"{cap.name}_vs_{obstacle.name}")
    return CollisionReport(collides=len(reasons) > 0, reasons=tuple(reasons))
