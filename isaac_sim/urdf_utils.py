# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Utilities for preparing MyCobot 280 M5 URDF files for Isaac Sim import.

Adapted from ``spark_isaac_mycobot_demo`` lessons: package:// URIs and COLLADA
GUID materials break Isaac Sim 6.x import unless rewritten. See
``docs/legacy/v1_lessons_learned.md``.
"""
from __future__ import annotations

import re
import shutil
from pathlib import Path

# Elephant Robotics COLLADA exports use GUID material IDs that Isaac Sim 6.x
# cannot turn into USD prim names (import fails with getPrimNames(None, ...)).
_UUID_MATERIAL_ID = "a0000000-0000-0000-0000-000000000000"
_UUID_MATERIAL_SYMBOL = f"material-{_UUID_MATERIAL_ID}"
_UUID_EFFECT_ID = f"fx-{_UUID_MATERIAL_ID}"

PACKAGE_URI_PATTERN = re.compile(
    r'package://mycobot_description/urdf/mycobot_280_m5/([^"\']+)'
)

REVOLUTE_JOINT_NAMES: tuple[str, ...] = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)

ROBOT_PRIM_PATH = "/World/MyCobot280"


def resolve_mycobot_280_m5_package_uris(urdf_text: str) -> str:
    """Replace package:// mesh URIs with relative filenames for Isaac Sim."""

    def _replace(match: re.Match[str]) -> str:
        return match.group(1)

    return PACKAGE_URI_PATTERN.sub(_replace, urdf_text)


def replace_g_base_mesh_with_box(urdf_text: str) -> str:
    """Swap G_base.dae for a box primitive (Isaac Sim 6.x COLLADA workaround)."""
    return urdf_text.replace(
        '<mesh filename="G_base.dae"/>',
        '<box size="0.12 0.12 0.06"/>',
    )


def write_isaac_ready_urdf(
    source_urdf: Path,
    output_urdf: Path,
    *,
    mesh_dir: Path | None = None,
) -> Path:
    """Write a copy of the upstream URDF with Isaac Sim-friendly mesh paths."""
    if not source_urdf.is_file():
        raise FileNotFoundError(f"Source URDF not found: {source_urdf}")
    resolved_mesh_dir = mesh_dir or source_urdf.parent
    if not resolved_mesh_dir.is_dir():
        raise FileNotFoundError(f"Mesh directory not found: {resolved_mesh_dir}")

    urdf_text = source_urdf.read_text(encoding="utf-8")
    prepared = resolve_mycobot_280_m5_package_uris(urdf_text)
    prepared = replace_g_base_mesh_with_box(prepared)
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    output_urdf.write_text(prepared, encoding="utf-8")
    return output_urdf


def default_upstream_urdf(repo_root: Path) -> Path:
    """Vendor MyCobot 280 M5 URDF (meshes required for rendered Phase 1)."""
    return (
        repo_root
        / "third_party"
        / "mycobot_ros2"
        / "mycobot_description"
        / "urdf"
        / "mycobot_280_m5"
        / "mycobot_280_m5.urdf"
    )


def default_prepared_urdf(repo_root: Path) -> Path:
    """Isaac-ready URDF written under ``assets/robots/``."""
    return (
        repo_root
        / "assets"
        / "robots"
        / "mycobot_280_m5"
        / "mycobot_280_m5.urdf"
    )


def _safe_material_basename(dae_path: Path) -> str:
    stem = re.sub(r"[^A-Za-z0-9_]+", "_", dae_path.stem)
    return stem or "mesh"


def sanitize_collada_materials(dae_path: Path) -> bool:
    """Rewrite GUID-style COLLADA material IDs to Isaac Sim-safe names."""
    if not dae_path.is_file() or dae_path.suffix.lower() != ".dae":
        return False
    text = dae_path.read_text(encoding="utf-8")
    if _UUID_MATERIAL_ID not in text:
        return False
    safe_name = f"material_{_safe_material_basename(dae_path)}"
    safe_effect = f"fx_{safe_name}"
    replacements = (
        (_UUID_EFFECT_ID, safe_effect),
        (_UUID_MATERIAL_SYMBOL, safe_name),
        (_UUID_MATERIAL_ID, safe_name),
    )
    updated = text
    for old, new in replacements:
        updated = updated.replace(old, new)
    if updated == text:
        return False
    dae_path.write_text(updated, encoding="utf-8")
    return True


def sanitize_collada_materials_in_dir(mesh_dir: Path) -> list[Path]:
    """Sanitize every ``.dae`` under *mesh_dir* that uses GUID material IDs."""
    changed: list[Path] = []
    if not mesh_dir.is_dir():
        return changed
    for dae_path in sorted(mesh_dir.glob("*.dae")):
        if sanitize_collada_materials(dae_path):
            changed.append(dae_path)
    return changed


def prepare_robot_assets(repo_root: Path, upstream_urdf: Path | None = None) -> Path:
    """Copy meshes + write Isaac-ready URDF; return prepared URDF path."""
    upstream = (upstream_urdf or default_upstream_urdf(repo_root)).resolve()
    if not upstream.is_file():
        raise FileNotFoundError(
            f"Upstream URDF missing: {upstream}\n"
            "Run: ./scripts/download_mycobot_ros2.sh"
        )
    mesh_source_dir = upstream.parent
    prepared_urdf = default_prepared_urdf(repo_root)
    prepared_dir = prepared_urdf.parent
    if prepared_dir.exists():
        shutil.rmtree(prepared_dir)
    prepared_dir.mkdir(parents=True, exist_ok=True)
    for mesh_path in mesh_source_dir.iterdir():
        if mesh_path.is_file() and mesh_path.suffix.lower() in {".dae", ".png"}:
            shutil.copy2(mesh_path, prepared_dir / mesh_path.name)
    write_isaac_ready_urdf(upstream, prepared_urdf, mesh_dir=prepared_dir)
    sanitize_collada_materials_in_dir(prepared_dir)
    return prepared_urdf
