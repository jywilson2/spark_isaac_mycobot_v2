# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Transparent USD overlay of mesh-fitted robot collision spheres (GUI debug).

Why this exists
---------------
Operators cannot see cuRobo's collision envelope on the mesh alone. This draws
the same spheres from ``mycobot_280_collision_spheres.yaml`` as translucent
UsdGeom.Sphere prims under ``/World/CollisionSpheresDebug``, updated each
servo tick from URDF FK.

Tip-omit links (``joint6_flange``) use a distinct tint so spheres-ON vs tip-omit
legs are visually distinguishable. See ``spec.md`` Phase 2 and
``--show-collision-spheres`` on ``isaac_sim/run_ik_viz.py``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from residual_adaptive_ik.planning.collision_sphere_world import (
    WorldCollisionSphere,
    collision_spheres_in_base_frame,
)

# Prim root for all debug spheres (removed/recreated when enabled).
COLLISION_SPHERE_ROOT = "/World/CollisionSpheresDebug"

# RGBA-ish: RGB for UsdPreviewSurface; opacity separate (0=invisible, 1=solid).
DEFAULT_ARM_RGB = (0.95, 0.55, 0.12)  # amber — proximal / spheres-ON
DEFAULT_TIP_RGB = (0.15, 0.75, 0.95)  # cyan — tip-omit links
DEFAULT_OPACITY = 0.35


@dataclass
class CollisionSphereVisualizer:
    """Create and update translucent collision-sphere overlays on a USD stage."""

    stage: Any
    opacity: float = DEFAULT_OPACITY
    arm_rgb: tuple[float, float, float] = DEFAULT_ARM_RGB
    tip_rgb: tuple[float, float, float] = DEFAULT_TIP_RGB
    _prim_paths: list[str] = field(default_factory=list, repr=False)
    _ready: bool = False

    def ensure_prims(self, spheres: list[WorldCollisionSphere]) -> None:
        """Define one UsdGeom.Sphere per fitted sphere (idempotent layout)."""
        from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: WPS433

        root = Sdf.Path(COLLISION_SPHERE_ROOT)
        if self.stage.GetPrimAtPath(root).IsValid():
            self.stage.RemovePrim(root)
        UsdGeom.Xform.Define(self.stage, root)
        self._prim_paths = []

        for i, sph in enumerate(spheres):
            path = root.AppendPath(f"s_{i:03d}_{sph.link}_{sph.index}")
            geom = UsdGeom.Sphere.Define(self.stage, path)
            geom.GetRadiusAttr().Set(float(sph.radius_m))
            xform = UsdGeom.Xformable(geom)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(
                Gf.Vec3d(
                    float(sph.center_m[0]),
                    float(sph.center_m[1]),
                    float(sph.center_m[2]),
                )
            )
            rgb = self.tip_rgb if sph.is_tip else self.arm_rgb
            mat_path = path.AppendPath("Material")
            mat = UsdShade.Material.Define(self.stage, mat_path)
            shader = UsdShade.Shader.Define(
                self.stage, mat.GetPath().AppendPath("Shader")
            )
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*rgb)
            )
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(rgb[0] * 0.15, rgb[1] * 0.15, rgb[2] * 0.15)
            )
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
                float(np.clip(self.opacity, 0.05, 1.0))
            )
            # Enable transparency in the viewport.
            shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
            # Also set gprim displayOpacity (some Kit viewports honor this).
            try:
                UsdGeom.Gprim(geom).CreateDisplayOpacityAttr().Set(
                    [float(np.clip(self.opacity, 0.05, 1.0))]
                )
            except Exception:
                pass
            mat.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface"
            )
            UsdShade.MaterialBindingAPI(geom).Bind(mat)
            self._prim_paths.append(str(path))
        self._ready = True

    def update(self, q_rad: np.ndarray) -> None:
        """Reposition all spheres for joint configuration ``q_rad`` (radians)."""
        from pxr import Gf, UsdGeom  # noqa: WPS433

        spheres = collision_spheres_in_base_frame(q_rad)
        if not self._ready or len(spheres) != len(self._prim_paths):
            self.ensure_prims(spheres)
        for path, sph in zip(self._prim_paths, spheres, strict=True):
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            geom = UsdGeom.Sphere(prim)
            geom.GetRadiusAttr().Set(float(sph.radius_m))
            xform = UsdGeom.Xformable(geom)
            ops = xform.GetOrderedXformOps()
            if ops:
                ops[0].Set(
                    Gf.Vec3d(
                        float(sph.center_m[0]),
                        float(sph.center_m[1]),
                        float(sph.center_m[2]),
                    )
                )

    def clear(self) -> None:
        """Remove the debug overlay from the stage."""
        from pxr import Sdf  # noqa: WPS433

        root = Sdf.Path(COLLISION_SPHERE_ROOT)
        if self.stage.GetPrimAtPath(root).IsValid():
            self.stage.RemovePrim(root)
        self._prim_paths = []
        self._ready = False
