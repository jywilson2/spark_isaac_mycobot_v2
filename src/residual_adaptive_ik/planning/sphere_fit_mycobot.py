# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Fit cuRobo collision spheres to MyCobot 280 link meshes (host / CUDA).

Why mesh fitting
----------------
Hand-tuned spheres were either too fat (false self-collisions) or too thin
(EE swept through the volumetric IK target). cuRobo's
``Mesh.get_bounding_spheres`` / ``fit_spheres_to_mesh`` approximates each link
mesh so robot↔target checks use volume, not a tip point.

The fitted YAML is committed under ``configs/planning/curobo/`` so CI and
container NumPy paths do not need trimesh/cuRobo. Regenerate on the host:

  ./scripts/host/spark_host_exec.sh ./scripts/host/fit_mycobot_collision_spheres.sh

See ``spec.md`` Phase 2 and ``curobo.geom.sphere_fit``.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml

from residual_adaptive_ik.kinematics.urdf_model import (
    DEFAULT_BASE_LINK,
    DEFAULT_EE_LINK,
    _repo_root,
    resolve_urdf_path,
)

# Collision links used by MotionGen (link frame names in the vendor URDF).
DEFAULT_COLLISION_LINKS: tuple[str, ...] = (
    "g_base",
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint6_flange",
)

DEFAULT_SPHERES_YAML = (
    _repo_root() / "configs" / "planning" / "curobo" / "mycobot_280_collision_spheres.yaml"
)

# Tip link may occupy the goal marker volume at contact; keep proximal EE
# (joint6) fitted spheres so the EE *body* still cannot bury into the target.
DEFAULT_TIP_LINKS_IGNORE_TARGET: tuple[str, ...] = ("joint6_flange",)


def _mesh_dir_for_urdf(urdf_path: Path) -> Path:
    """Return the directory containing MyCobot ``*.dae`` meshes."""
    # Vendor layout: .../urdf/mycobot_280_m5/mycobot_280_m5.urdf
    if (urdf_path.parent / "joint1.dae").is_file():
        return urdf_path.parent
    root = _repo_root()
    candidate = (
        root
        / "third_party"
        / "mycobot_ros2"
        / "mycobot_description"
        / "urdf"
        / "mycobot_280_m5"
    )
    if (candidate / "joint1.dae").is_file():
        return candidate
    raise FileNotFoundError(
        f"MyCobot meshes not found next to {urdf_path} or under third_party/"
    )


def _link_mesh_filenames(urdf_text: str) -> dict[str, str]:
    """Map link name → first mesh basename from URDF (visual or collision)."""
    out: dict[str, str] = {}
    for match in re.finditer(
        r'<link\s+name="([^"]+)">([\s\S]*?)</link>', urdf_text, flags=re.IGNORECASE
    ):
        name, body = match.group(1), match.group(2)
        files = re.findall(r'filename="([^"]+)"', body)
        if not files:
            continue
        # Prefer collision mesh if tagged; else first filename.
        chosen = files[0]
        for f in files:
            if "collision" in body[max(0, body.find(f) - 80) : body.find(f)].lower():
                chosen = f
                break
        out[name] = Path(chosen.replace("package://mycobot_description/", "")).name
        # package://mycobot_description/urdf/mycobot_280_m5/joint1.dae → joint1.dae
        if "/" in chosen:
            out[name] = chosen.rsplit("/", 1)[-1]
    return out


def fit_link_collision_spheres(
    *,
    urdf_path: Path | None = None,
    collision_links: tuple[str, ...] = DEFAULT_COLLISION_LINKS,
    n_spheres_per_link: int = 16,
    surface_sphere_radius_m: float = 0.005,
) -> dict[str, list[dict[str, Any]]]:
    """Fit spheres to each link mesh. Requires cuRobo + trimesh (host Isaac python).

    Returns cuRobo ``collision_spheres`` dict: link → [{center, radius}, ...].
    Centers are in the **link frame** (meters).

    Note: Elephant ``G_base.dae`` is authored in millimeters; other links are in
    meters. Meshes whose longest extent exceeds ``0.5`` m are scaled by
    ``0.001`` before fitting.
    """
    import numpy as np
    import trimesh
    from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh

    urdf = resolve_urdf_path(urdf_path)
    mesh_dir = _mesh_dir_for_urdf(urdf)
    link_files = _link_mesh_filenames(urdf.read_text(encoding="utf-8"))
    spheres: dict[str, list[dict[str, Any]]] = {}

    for link in collision_links:
        fname = link_files.get(link)
        if fname is None:
            raise KeyError(f"No mesh filename for link {link!r} in {urdf}")
        mesh_path = mesh_dir / fname
        if not mesh_path.is_file():
            raise FileNotFoundError(mesh_path)
        loaded = trimesh.load(str(mesh_path.resolve()), force="mesh")
        if isinstance(loaded, trimesh.Scene):
            geoms = [g for g in loaded.geometry.values() if isinstance(g, trimesh.Trimesh)]
            if not geoms:
                raise RuntimeError(f"No Trimesh geometry in {mesh_path}")
            tm = trimesh.util.concatenate(geoms)
        else:
            tm = loaded
        # Heuristic: G_base.dae is mm; joint*.dae are meters.
        if float(np.max(tm.extents)) > 0.5:
            tm = tm.copy()
            tm.apply_scale(0.001)
        pts, radii = fit_spheres_to_mesh(
            tm,
            n_spheres=int(n_spheres_per_link),
            surface_sphere_radius=float(surface_sphere_radius_m),
            fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
        )
        if pts is None or len(pts) == 0:
            # Fallback: one sphere at centroid.
            c = np.asarray(tm.centroid, dtype=float).reshape(3)
            r = float(max(np.max(tm.extents) * 0.25, surface_sphere_radius_m))
            spheres[link] = [{"center": c.tolist(), "radius": r}]
            continue
        link_spheres: list[dict[str, Any]] = []
        for i in range(len(pts)):
            link_spheres.append(
                {
                    "center": [float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2])],
                    "radius": float(radii[i]),
                }
            )
        spheres[link] = link_spheres
    return spheres


def adjacent_self_collision_ignore(
    links: tuple[str, ...] = DEFAULT_COLLISION_LINKS,
) -> dict[str, list[str]]:
    """Ignore only immediate neighbors (mesh-fitted spheres need less padding)."""
    ignore: dict[str, list[str]] = {}
    for i, link in enumerate(links):
        nbrs: list[str] = []
        if i > 0:
            nbrs.append(links[i - 1])
        if i + 1 < len(links):
            nbrs.append(links[i + 1])
        ignore[link] = nbrs
    return ignore


def write_collision_spheres_yaml(
    spheres: dict[str, list[dict[str, Any]]],
    path: Path | None = None,
    *,
    self_collision_ignore: dict[str, list[str]] | None = None,
    tip_links_ignore_target: tuple[str, ...] = DEFAULT_TIP_LINKS_IGNORE_TARGET,
    n_spheres_per_link: int = 12,
    surface_sphere_radius_m: float = 0.002,
) -> Path:
    """Write fitted spheres + metadata for ``build_mycobot_curobo_robot_cfg``."""
    out = Path(path) if path is not None else DEFAULT_SPHERES_YAML
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "generator": "curobo.geom.sphere_fit.fit_spheres_to_mesh",
        "fit_type": "VOXEL_VOLUME_SAMPLE_SURFACE",
        "n_spheres_per_link": int(n_spheres_per_link),
        "surface_sphere_radius_m": float(surface_sphere_radius_m),
        "base_link": DEFAULT_BASE_LINK,
        "ee_link": DEFAULT_EE_LINK,
        "tip_links_ignore_target": list(tip_links_ignore_target),
        "self_collision_ignore": self_collision_ignore
        or adjacent_self_collision_ignore(),
        "collision_spheres": spheres,
    }
    out.write_text(
        "# Auto-generated by fit_mycobot_collision_spheres (cuRobo mesh fit).\n"
        "# Regenerate: ./scripts/host/fit_mycobot_collision_spheres.sh\n"
        + yaml.safe_dump(payload, default_flow_style=False, sort_keys=False),
        encoding="utf-8",
    )
    return out


def load_collision_spheres_yaml(path: Path | None = None) -> dict[str, Any]:
    """Load committed / generated collision spheres YAML."""
    cfg_path = Path(path) if path is not None else DEFAULT_SPHERES_YAML
    if not cfg_path.is_file():
        raise FileNotFoundError(
            f"Missing fitted spheres YAML: {cfg_path}. "
            "Run scripts/host/fit_mycobot_collision_spheres.sh on the host."
        )
    with cfg_path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if "collision_spheres" not in data:
        raise ValueError(f"{cfg_path} missing collision_spheres")
    return dict(data)
