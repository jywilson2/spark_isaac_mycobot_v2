# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Isaac-ready URDF prep tests (no Isaac Sim required)."""
from __future__ import annotations

from pathlib import Path

from isaac_sim.urdf_utils import (
    REVOLUTE_JOINT_NAMES,
    replace_g_base_mesh_with_box,
    resolve_mycobot_280_m5_package_uris,
    write_isaac_ready_urdf,
)

REPO = Path(__file__).resolve().parents[1]
VENDOR_URDF = (
    REPO
    / "third_party"
    / "mycobot_ros2"
    / "mycobot_description"
    / "urdf"
    / "mycobot_280_m5"
    / "mycobot_280_m5.urdf"
)


def test_revolute_joint_names_match_fk_order():
    assert len(REVOLUTE_JOINT_NAMES) == 6
    assert REVOLUTE_JOINT_NAMES[0] == "joint2_to_joint1"


def test_package_uri_rewrite():
    raw = '<mesh filename="package://mycobot_description/urdf/mycobot_280_m5/joint1.dae"/>'
    out = resolve_mycobot_280_m5_package_uris(raw)
    assert 'package://' not in out
    assert 'filename="joint1.dae"' in out


def test_g_base_box_replacement():
    raw = '<mesh filename="G_base.dae"/>'
    out = replace_g_base_mesh_with_box(raw)
    assert "G_base.dae" not in out
    assert "<box" in out


def test_write_isaac_ready_urdf_when_vendor_present(tmp_path):
    if not VENDOR_URDF.is_file():
        import pytest

        pytest.skip("vendor mycobot_ros2 URDF not available")
    out = tmp_path / "prepared.urdf"
    write_isaac_ready_urdf(VENDOR_URDF, out, mesh_dir=VENDOR_URDF.parent)
    text = out.read_text()
    assert "package://mycobot_description" not in text
    assert "G_base.dae" not in text
