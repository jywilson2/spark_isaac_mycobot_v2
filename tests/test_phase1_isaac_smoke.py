# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Optional host Isaac Sim Phase 1 smoke (gated; CI = headless).

Enable with::

    # CI / remote PR / agent (headless Kit):
    SPARK_RUN_ISAAC_SMOKE=1 pytest tests/test_phase1_isaac_smoke.py -q

    # From the Isaac ROS / Cursor container (delegates via nsenter; headless):
    SPARK_RUN_ISAAC_SMOKE=1 ./scripts/host/spark_host_exec.sh \\
      ./scripts/host/smoke_phase1_isaac.sh

On a DGX Spark host with Isaac Sim, after headless succeeds, also run GUI smoke
from a native desktop session (not required for remote GitHub PR CI)::

    ./scripts/host/smoke_phase1_isaac.sh --gui

Requires Isaac Sim ``python.sh``. Skips when the env gate is unset.
See ``spec.md`` Phase 1 Acceptance Criteria item 7.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SMOKE = REPO / "scripts" / "host" / "smoke_phase1_isaac.sh"
HOST_EXEC = REPO / "scripts" / "host" / "spark_host_exec.sh"


@pytest.mark.isaac
def test_phase1_isaac_host_smoke():
    """CI gate: headless host smoke only (never passes --gui)."""
    if os.environ.get("SPARK_RUN_ISAAC_SMOKE", "0") != "1":
        pytest.skip("Set SPARK_RUN_ISAAC_SMOKE=1 to run host Isaac Phase 1 smoke")
    if not SMOKE.is_file():
        pytest.fail(f"missing smoke script: {SMOKE}")

    env = os.environ.copy()
    env.setdefault("PHASE1_SMOKE_N_POSES", "48")
    env.setdefault("PHASE1_SMOKE_VISUALIZE", "12")
    env.setdefault("PHASE1_SMOKE_HOLD_S", "0.15")
    env.setdefault("SPARK_HOST_USER", "jywilson")
    env.setdefault("ISAACSIM_PATH", f"/home/{env['SPARK_HOST_USER']}/isaacsim")

    in_docker = Path("/.dockerenv").is_file()
    if in_docker:
        cmd = ["bash", str(HOST_EXEC), "./scripts/host/smoke_phase1_isaac.sh"]
    else:
        cmd = ["bash", str(SMOKE)]

    proc = subprocess.run(cmd, cwd=str(REPO), env=env, check=False)
    assert proc.returncode == 0, f"Isaac Phase 1 smoke failed (exit {proc.returncode})"


def test_smoke_script_exists_and_is_documented():
    assert SMOKE.is_file()
    text = SMOKE.read_text(encoding="utf-8")
    assert "run_phase1_isaac.sh" in text
    assert "--headless" in text
    assert "--gui" in text
    assert HOST_EXEC.is_file()


def test_urdf_importer_sets_drive_gains_in_source():
    """Regression: do not ship an importer config that omits stiffness/damping."""
    src = (REPO / "isaac_sim" / "urdf_import.py").read_text(encoding="utf-8")
    assert "override_joint_stiffness" in src
    assert "override_joint_damping" in src
    assert "load_joint_drive_gains" in src
    assert (REPO / "configs" / "robot" / "joint_drives.yaml").is_file()


def test_spark_host_exec_drops_to_host_user_for_gui():
    """v1 only set HOME/USER as root; v2 must runuser so X11 cookies work."""
    src = (REPO / "scripts" / "host" / "spark_host_exec.sh").read_text(encoding="utf-8")
    assert "runuser" in src
    assert "SPARK_HOST_RUN_AS_USER" in src


def test_smoke_gui_passes_auto_exit_by_default():
    text = SMOKE.read_text(encoding="utf-8")
    assert "--auto-exit" in text
    assert "PHASE1_SMOKE_KEEP_GUI_OPEN" in text
