# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""URDF import helpers for Isaac Sim 5.x/6.x standalone scripts.

Must be executed with ``${ISAACSIM_PATH}/python.sh`` on the host.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from isaacsim import SimulationApp

URDF_IMPORTER_EXTENSION = "isaacsim.asset.importer.urdf"
SCENE_OPTIMIZER_EXTENSION = "omni.scene.optimizer.core"
ROBOT_SCHEMA_EXTENSION = "isaacsim.robot.schema"

_REQUIRED_EXTENSIONS = (
    SCENE_OPTIMIZER_EXTENSION,
    ROBOT_SCHEMA_EXTENSION,
    URDF_IMPORTER_EXTENSION,
)


def _wait_for_extension(extension_id: str, timeout_s: float = 30.0) -> None:
    import omni.kit.app  # noqa: WPS433

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if extension_manager.is_extension_enabled(extension_id):
            return
        time.sleep(0.1)
    raise RuntimeError(
        f"Timed out waiting for Isaac Sim extension {extension_id!r} to enable."
    )


def enable_urdf_importer_extensions(
    simulation_app: SimulationApp,
    update_cycles: int = 30,
) -> None:
    """Load extensions required by the Isaac Sim 6 URDF importer."""
    import omni.kit.app  # noqa: WPS433

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    for extension_id in _REQUIRED_EXTENSIONS:
        extension_manager.set_extension_enabled_immediate(extension_id, True)
    for _ in range(update_cycles):
        simulation_app.update()
    for extension_id in _REQUIRED_EXTENSIONS:
        _wait_for_extension(extension_id)


def resolve_imported_usd_path(
    imported_path: str | None,
    output_usd: Path,
    search_dir: Path,
) -> Path:
    """Resolve the robot USD file produced by URDF import."""
    candidates: list[Path] = []
    if imported_path:
        candidates.append(Path(imported_path))
    candidates.append(output_usd)
    candidates.extend(
        sorted(search_dir.glob("*.usd*"), key=lambda path: path.stat().st_mtime, reverse=True)
    )
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_file():
            return resolved
    raise RuntimeError(
        "URDF import did not produce a USD file. "
        f"Checked import output {imported_path!r} and directory {search_dir}"
    )


def _build_urdf_importer_config(prepared_urdf: Path, output_usd: Path) -> Any:
    from isaacsim.asset.importer.urdf.impl import URDFImporterConfig  # noqa: WPS433

    from isaac_sim.joint_drives import load_joint_drive_gains  # noqa: WPS433

    # Vendor URDF omits drive gains; Isaac warns unless overridden (Acceptance #8).
    # Gains are derived from published payload/reach/accuracy — see joint_drives.yaml.
    gains = load_joint_drive_gains()

    config = URDFImporterConfig()
    config.urdf_path = str(prepared_urdf.resolve())
    config.usd_path = str(output_usd.parent.resolve())
    config.merge_mesh = False
    config.collision_from_visuals = False
    config.allow_self_collision = False
    config.fix_base = True
    config.joint_target_type = "position"
    config.override_joint_stiffness = float(gains.stiffness)  # N·m/rad
    config.override_joint_damping = float(gains.damping)  # N·m·s/rad
    return config


def import_urdf_to_usd(
    prepared_urdf: Path,
    output_usd: Path,
    simulation_app: SimulationApp,
) -> Path:
    """Convert URDF to a standalone USD file on disk."""
    from isaacsim.asset.importer.urdf.impl import URDFImporter  # noqa: WPS433

    enable_urdf_importer_extensions(simulation_app)
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    import_config = _build_urdf_importer_config(prepared_urdf, output_usd)
    importer = URDFImporter(import_config)
    imported_path = importer.import_urdf()
    if not imported_path:
        raise RuntimeError(f"URDFImporter.import_urdf() returned no path for {prepared_urdf}")
    return resolve_imported_usd_path(imported_path, output_usd, output_usd.parent)


def add_robot_reference_to_stage(robot_usd: Path, dest_prim_path: str) -> str:
    """Reference an imported robot USD under dest_prim_path in the active stage."""
    import omni.usd  # noqa: WPS433
    from pxr import Sdf, UsdGeom  # noqa: WPS433

    stage = omni.usd.get_context().get_stage()
    world_path = Sdf.Path("/World")
    if not stage.GetPrimAtPath(world_path).IsValid():
        UsdGeom.Xform.Define(stage, world_path)

    dest_path = Sdf.Path(dest_prim_path)
    if stage.GetPrimAtPath(dest_path).IsValid():
        stage.RemovePrim(dest_path)

    robot_prim = stage.DefinePrim(dest_path, "Xform")
    robot_prim.GetReferences().AddReference(str(robot_usd.resolve()))
    return str(dest_path)
