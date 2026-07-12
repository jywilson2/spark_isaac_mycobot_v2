# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Optional host Isaac Sim IK viz smoke (gated; CI = headless).

Covers Phase 1 metrics + Phase 2 planning in one Kit run.

Enable with::

    # CI / remote PR / agent (headless Kit):
    SPARK_RUN_ISAAC_SMOKE=1 pytest tests/test_isaac_viz_smoke.py -q

    # From the Isaac ROS / Cursor container (delegates via nsenter; headless):
    SPARK_RUN_ISAAC_SMOKE=1 ./scripts/host/spark_host_exec.sh \\
      ./scripts/host/smoke_isaac_viz.sh

On a DGX Spark host with Isaac Sim, after headless succeeds, also run GUI smoke
from a native desktop session (not required for remote GitHub PR CI)::

    ./scripts/host/smoke_isaac_viz.sh --gui

Requires Isaac Sim ``python.sh``. Skips when the env gate is unset.
See ``spec.md`` Acceptance Criteria item 7.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SMOKE = REPO / "scripts" / "host" / "smoke_isaac_viz.sh"
HOST_EXEC = REPO / "scripts" / "host" / "spark_host_exec.sh"


@pytest.mark.isaac
def test_isaac_viz_host_smoke():
    """CI gate: headless host smoke only (never passes --gui)."""
    if os.environ.get("SPARK_RUN_ISAAC_SMOKE", "0") != "1":
        pytest.skip("Set SPARK_RUN_ISAAC_SMOKE=1 to run host Isaac viz smoke")
    if not SMOKE.is_file():
        pytest.fail(f"missing smoke script: {SMOKE}")

    env = os.environ.copy()
    env.setdefault("ISAAC_VIZ_SMOKE_N_POSES", "48")
    env.setdefault("ISAAC_VIZ_SMOKE_VISUALIZE", "12")
    env.setdefault("ISAAC_VIZ_SMOKE_HOLD_S", "0.15")

    if (REPO / ".dockerenv").exists() or Path("/.dockerenv").is_file():
        cmd = ["bash", str(HOST_EXEC), "./scripts/host/smoke_isaac_viz.sh"]
    else:
        cmd = ["bash", str(SMOKE)]
    proc = subprocess.run(cmd, cwd=str(REPO), env=env, check=False)
    assert proc.returncode == 0, f"Isaac viz smoke failed: {cmd}"


def test_smoke_isaac_viz_script_exists_and_documents_policy():
    assert SMOKE.is_file()
    text = SMOKE.read_text(encoding="utf-8")
    assert "run_isaac_viz.sh" in text
    assert "--gui" in text
    assert "headless" in text.lower()
    assert "override_joint_stiffness" in (
        REPO / "isaac_sim" / "urdf_import.py"
    ).read_text(encoding="utf-8") or "joint_drives" in text
    assert "ISAAC_VIZ_SMOKE_KEEP_GUI_OPEN" in text
    assert "PHASE1_SMOKE_KEEP_GUI_OPEN" in text  # legacy alias
    assert "--auto-exit" in text
    assert "--reset-to-home" in text


def test_legacy_smoke_phase1_alias_forwards():
    legacy = REPO / "scripts" / "host" / "smoke_phase1_isaac.sh"
    assert legacy.is_file()
    text = legacy.read_text(encoding="utf-8")
    assert "smoke_isaac_viz.sh" in text
    assert "deprecated" in text.lower()
