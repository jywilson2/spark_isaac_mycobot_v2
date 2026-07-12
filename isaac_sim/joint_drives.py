# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Load Isaac Sim joint position-drive gains from ``configs/robot/joint_drives.yaml``.

Elephant Robotics does not publish stiffness/damping. Values are derived from
published payload, reach, and positioning accuracy — see the YAML header and
``spec.md`` / ``docs/isaac_sim_host.md``.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_JOINT_DRIVES_PATH = _REPO_ROOT / "configs" / "robot" / "joint_drives.yaml"


@dataclass(frozen=True)
class JointDriveGains:
    """Uniform revolute position-drive gains for the URDF importer (SI)."""

    stiffness_nm_per_rad: float
    damping_nm_s_per_rad: float
    config_path: Path

    @property
    def stiffness(self) -> float:
        """Alias for importer ``override_joint_stiffness`` (N·m/rad)."""
        return self.stiffness_nm_per_rad

    @property
    def damping(self) -> float:
        """Alias for importer ``override_joint_damping`` (N·m·s/rad)."""
        return self.damping_nm_s_per_rad


def load_joint_drive_gains(path: Path | None = None) -> JointDriveGains:
    """Load drive gains from YAML.

    Raises:
        FileNotFoundError: config missing
        KeyError / TypeError / ValueError: malformed or non-positive gains
    """
    cfg_path = Path(path) if path is not None else DEFAULT_JOINT_DRIVES_PATH
    if not cfg_path.is_file():
        raise FileNotFoundError(f"joint drive config not found: {cfg_path}")
    with cfg_path.open(encoding="utf-8") as handle:
        raw: dict[str, Any] = dict(yaml.safe_load(handle) or {})
    stiffness = float(raw["stiffness_nm_per_rad"])
    damping = float(raw["damping_nm_s_per_rad"])
    if stiffness <= 0.0 or damping <= 0.0:
        raise ValueError(
            f"joint drive gains must be positive; got K={stiffness}, D={damping} from {cfg_path}"
        )
    return JointDriveGains(
        stiffness_nm_per_rad=stiffness,
        damping_nm_s_per_rad=damping,
        config_path=cfg_path.resolve(),
    )


def derive_joint_drive_gains(
    *,
    working_radius_m: float = 0.280,
    payload_kg: float = 0.250,
    arm_mass_kg: float = 0.850,
    positioning_accuracy_m: float = 0.0005,
    gravity_m_s2: float = 9.81,
    arm_com_fraction: float = 0.5,
    arm_reach_fraction: float = 0.5,
    inertia_link_length_m: float = 0.20,
    damping_ratio_zeta: float = 1.2,
) -> tuple[float, float]:
    """Recompute K, D from mechanical specs (for unit tests / docs).

    Returns:
        ``(stiffness_nm_per_rad, damping_nm_s_per_rad)``
    """
    tau_payload = payload_kg * gravity_m_s2 * working_radius_m
    tau_arm = (
        arm_com_fraction * arm_mass_kg * gravity_m_s2 * (arm_reach_fraction * working_radius_m)
    )
    tau_grav = tau_payload + tau_arm
    delta_q = positioning_accuracy_m / working_radius_m
    stiffness = tau_grav / delta_q
    inertia_payload = payload_kg * working_radius_m * working_radius_m
    inertia_arm = (1.0 / 3.0) * arm_mass_kg * inertia_link_length_m * inertia_link_length_m
    inertia = inertia_payload + inertia_arm
    damping_critical = 2.0 * (stiffness * inertia) ** 0.5
    damping = damping_ratio_zeta * damping_critical
    return float(stiffness), float(damping)
