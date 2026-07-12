# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Contract tests for CI vs Spark verification script."""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "run_verification.sh"


def test_run_verification_script_exists_and_documents_modes():
    assert SCRIPT.is_file()
    text = SCRIPT.read_text(encoding="utf-8")
    assert "ci)" in text
    assert "spark)" in text
    assert "smoke_isaac_viz.sh --gui" in text
    assert "Acceptance #7" in text
    assert "spark_preflight" in text
    assert "ISAAC_VIZ_SMOKE_HEADLESS_VISUALIZE" in text or "ISAAC_VIZ_SMOKE_VISUALIZE" in text
    assert "ISAAC_VIZ_SMOKE_GUI_VISUALIZE" in text
    assert "install_curobo.sh" in text
    assert (REPO / "scripts" / "run_phase2_geometry.sh").is_file()
    assert (REPO / "scripts" / "run_phase3_supervised.sh").is_file()
    assert (REPO / "scripts" / "run_phase4_sac.sh").is_file()


def test_spec_and_cursorrules_point_at_run_verification():
    spec = (REPO / "spec.md").read_text(encoding="utf-8")
    rules = (REPO / ".cursorrules").read_text(encoding="utf-8")
    assert "run_verification.sh" in spec
    assert "run_verification.sh" in rules
    assert "./scripts/run_verification.sh ci" in spec
    assert "./scripts/run_verification.sh spark" in spec
