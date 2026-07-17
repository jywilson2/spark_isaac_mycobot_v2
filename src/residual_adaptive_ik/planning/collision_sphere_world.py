# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Place mesh-fitted collision spheres in the robot base frame (meters).

Why this exists
---------------
cuRobo plans with link-local spheres from
``configs/planning/curobo/mycobot_280_collision_spheres.yaml``. GUI debug needs
those same spheres in **world / base** coordinates so operators can see the
collision envelope (spheres-ON vs tip-omit) overlaid on the arm.

Pure NumPy + URDF FK — no Kit / cuRobo required for unit tests. See ``spec.md``
Phase 2 geometry and ``isaac_sim/collision_sphere_viz.py``.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from residual_adaptive_ik.kinematics.urdf_model import UrdfKinematicModel, get_default_model
from residual_adaptive_ik.planning.sphere_fit_mycobot import load_collision_spheres_yaml


@dataclass(frozen=True)
class WorldCollisionSphere:
    """One fitted collision sphere expressed in the robot base frame."""

    link: str
    index: int
    center_m: np.ndarray
    radius_m: float
    is_tip: bool

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "center_m", np.asarray(self.center_m, dtype=float).reshape(3)
        )


def tip_link_names(spheres_data: dict[str, Any] | None = None) -> frozenset[str]:
    """Links whose spheres are omitted during tip-omit contact planning."""
    data = spheres_data if spheres_data is not None else load_collision_spheres_yaml()
    raw = data.get("tip_links_ignore_target") or ("joint6_flange",)
    return frozenset(str(x) for x in raw)


def collision_spheres_in_base_frame(
    q_rad: np.ndarray,
    *,
    model: UrdfKinematicModel | None = None,
    spheres_data: dict[str, Any] | None = None,
    spheres_yaml: Path | None = None,
) -> list[WorldCollisionSphere]:
    """Transform link-local fitted spheres into the robot base frame.

    Parameters
    ----------
    q_rad:
        Joint angles (radians), length = model DOF.
    spheres_data / spheres_yaml:
        Optional override; default loads the committed mesh-fit YAML.
    """
    mdl = model or get_default_model()
    data = spheres_data
    if data is None:
        data = load_collision_spheres_yaml(spheres_yaml)
    link_spheres = data.get("collision_spheres") or {}
    tips = tip_link_names(data)
    T_by_link = mdl.link_transforms(np.asarray(q_rad, dtype=float).reshape(-1))
    out: list[WorldCollisionSphere] = []
    for link, entries in link_spheres.items():
        T = T_by_link.get(str(link))
        if T is None:
            continue
        R = T[:3, :3]
        p = T[:3, 3]
        for i, entry in enumerate(entries):
            c_local = np.asarray(entry["center"], dtype=float).reshape(3)
            r = float(entry["radius"])
            c_world = R @ c_local + p
            out.append(
                WorldCollisionSphere(
                    link=str(link),
                    index=int(i),
                    center_m=c_world,
                    radius_m=r,
                    is_tip=str(link) in tips,
                )
            )
    return out
