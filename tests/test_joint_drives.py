# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Unit tests for derived MyCobot joint drive gains (no Isaac Sim required)."""
from __future__ import annotations

from pathlib import Path

from isaac_sim.joint_drives import (
    DEFAULT_JOINT_DRIVES_PATH,
    derive_joint_drive_gains,
    load_joint_drive_gains,
)

REPO = Path(__file__).resolve().parents[1]


def test_joint_drives_yaml_exists_and_loads():
    assert DEFAULT_JOINT_DRIVES_PATH.is_file()
    gains = load_joint_drive_gains()
    assert gains.stiffness_nm_per_rad == 710.0
    assert gains.damping_nm_s_per_rad == 11.3
    assert gains.stiffness == gains.stiffness_nm_per_rad
    assert gains.damping == gains.damping_nm_s_per_rad


def test_derived_gains_match_config_within_rounding():
    """Config values are rounded from the mechanical derivation."""
    k, d = derive_joint_drive_gains()
    assert abs(k - 710.0) < 5.0  # N·m/rad
    assert abs(d - 11.3) < 0.5  # N·m·s/rad


def test_stiffness_covers_payload_gravity_at_accuracy():
    """K · (ε/L) must meet or exceed rated payload gravity torque at full reach."""
    gains = load_joint_drive_gains()
    radius_m = 0.280
    payload_kg = 0.250
    accuracy_m = 0.0005
    g = 9.81
    tau_payload = payload_kg * g * radius_m
    delta_q = accuracy_m / radius_m
    assert gains.stiffness_nm_per_rad * delta_q >= tau_payload


def test_urdf_importer_loads_gains_from_config():
    """Importer source must call load_joint_drive_gains (not hard-coded placeholders)."""
    src = (REPO / "isaac_sim" / "urdf_import.py").read_text(encoding="utf-8")
    assert "load_joint_drive_gains" in src
    assert "override_joint_stiffness" in src
    assert "override_joint_damping" in src
    assert "200.0" not in src  # superseded placeholder
