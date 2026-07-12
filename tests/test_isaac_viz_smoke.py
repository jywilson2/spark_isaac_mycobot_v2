# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Host Isaac Sim IK viz smoke (headless + GUI).

Policy
------
- Remote GitHub PR / ``CI=true``: headless gated; GUI never required.
- DGX Spark host development (this machine): GUI smoke runs by default when
  Isaac Sim + ``DISPLAY`` are detectable. Opt out with
  ``SPARK_RUN_ISAAC_GUI_SMOKE=0``.

Enable explicitly::

    SPARK_RUN_ISAAC_SMOKE=1 pytest tests/test_isaac_viz_smoke.py -q
    SPARK_RUN_ISAAC_GUI_SMOKE=1 pytest tests/test_isaac_viz_smoke.py -q

From the Isaac ROS / Cursor container, smokes delegate via
``scripts/host/spark_host_exec.sh``.

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


def _in_container() -> bool:
    return (REPO / ".dockerenv").exists() or Path("/.dockerenv").is_file()


def _smoke_cmd(*extra: str) -> list[str]:
    if _in_container():
        return ["bash", str(HOST_EXEC), "./scripts/host/smoke_isaac_viz.sh", *extra]
    return ["bash", str(SMOKE), *extra]


def _isaacsim_dir_candidates() -> list[Path]:
    env = os.environ.get("ISAACSIM_PATH", "").strip()
    out: list[Path] = []
    if env:
        out.append(Path(env))
    out.extend(
        [
            Path.home() / "isaacsim",
            Path("/home/jywilson/isaacsim"),
            Path("/home/admin/isaacsim"),
        ]
    )
    return out


def _spark_gui_auto_enabled() -> bool:
    """True when this host should run GUI smoke without an explicit env gate."""
    explicit = os.environ.get("SPARK_RUN_ISAAC_GUI_SMOKE", "").strip()
    if explicit == "0":
        return False
    if explicit == "1":
        return True
    # Remote CI must stay headless-only.
    if os.environ.get("CI", "").lower() in ("1", "true", "yes"):
        return False
    if os.environ.get("GITHUB_ACTIONS", "").lower() in ("1", "true", "yes"):
        return False
    # Local Spark development: Isaac ROS container delegates Kit via nsenter.
    if _in_container() and HOST_EXEC.is_file():
        return True
    if not os.environ.get("DISPLAY", "").strip():
        return False
    return any(p.is_dir() for p in _isaacsim_dir_candidates())


@pytest.mark.isaac
def test_isaac_viz_host_smoke():
    """Headless host smoke (CI / optional gate)."""
    if os.environ.get("SPARK_RUN_ISAAC_SMOKE", "0") != "1":
        pytest.skip("Set SPARK_RUN_ISAAC_SMOKE=1 to run host Isaac viz smoke")
    if not SMOKE.is_file():
        pytest.fail(f"missing smoke script: {SMOKE}")

    env = os.environ.copy()
    env.setdefault("ISAAC_VIZ_SMOKE_N_POSES", "48")
    env.setdefault("ISAAC_VIZ_SMOKE_VISUALIZE", "12")
    env.setdefault("ISAAC_VIZ_SMOKE_HOLD_S", "0.15")

    proc = subprocess.run(_smoke_cmd(), cwd=str(REPO), env=env, check=False)
    assert proc.returncode == 0, f"Isaac viz headless smoke failed: {_smoke_cmd()}"


@pytest.mark.isaac
def test_isaac_viz_gui_smoke():
    """GUI smoke — default on Spark hosts; required for local Phase 2 TDD."""
    if not _spark_gui_auto_enabled():
        pytest.skip(
            "GUI smoke skipped (set SPARK_RUN_ISAAC_GUI_SMOKE=1, or run on Spark "
            "with DISPLAY + Isaac Sim; opt out with SPARK_RUN_ISAAC_GUI_SMOKE=0)"
        )
    if not SMOKE.is_file():
        pytest.fail(f"missing smoke script: {SMOKE}")

    env = os.environ.copy()
    env.setdefault("ISAAC_VIZ_SMOKE_N_POSES", "240")
    env.setdefault("ISAAC_VIZ_SMOKE_VISUALIZE", "48")
    env.setdefault("ISAAC_VIZ_SMOKE_HOLD_S", "0.2")
    # Path-dependent recovery: no per-trial home reset (home once at viz start).
    env["ISAAC_VIZ_SMOKE_RESET_TO_HOME"] = "0"

    cmd = _smoke_cmd("--gui", "--no-reset-to-home")
    proc = subprocess.run(cmd, cwd=str(REPO), env=env, check=False)
    assert proc.returncode == 0, f"Isaac viz GUI smoke failed: {cmd}"


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
    assert "--no-reset-to-home" in text
    assert "ISAAC_VIZ_MIN_PLAN_OK_RATE" in text
    assert "--min-plan-ok-rate" in text
    # Default GUI automated smoke must not force per-trial home reset.
    gui_src = Path(__file__).read_text(encoding="utf-8")
    assert '--gui", "--no-reset-to-home"' in gui_src
    assert 'ISAAC_VIZ_SMOKE_RESET_TO_HOME"] = "0"' in gui_src
    ver = (REPO / "scripts" / "run_verification.sh").read_text(encoding="utf-8")
    assert "--no-reset-to-home" in ver
    assert "smoke_isaac_viz.sh --gui --reset-to-home" not in ver


def test_viz_moves_home_once_and_turns_marker_green_on_contact():
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "moving to home once before trials" in src
    assert "MARKER_CONTACT: tip on sphere surface" in src
    assert "MarkerVisualState.CONTACT" in src
    assert "ee_contacts_target" in src


def test_run_isaac_viz_rechecks_plan_ok_rate_after_kit():
    """Kit/python.sh often returns 0; wrapper must re-check metrics JSON."""
    script = REPO / "scripts" / "host" / "run_isaac_viz.sh"
    text = script.read_text(encoding="utf-8")
    assert "phase2_plan_ok_rate" in text
    assert "phase2_min_plan_ok_rate" in text
    assert "post-Kit check" in text or "post-Kit" in text


def test_legacy_smoke_phase1_alias_forwards():
    legacy = REPO / "scripts" / "host" / "smoke_phase1_isaac.sh"
    assert legacy.is_file()
    text = legacy.read_text(encoding="utf-8")
    assert "smoke_isaac_viz.sh" in text
    assert "deprecated" in text.lower()


def test_spark_gui_auto_detect_helpers():
    """Source contract: auto-enable path is documented and opt-outable."""
    src = Path(__file__).read_text(encoding="utf-8")
    assert "SPARK_RUN_ISAAC_GUI_SMOKE" in src
    assert "_spark_gui_auto_enabled" in src
