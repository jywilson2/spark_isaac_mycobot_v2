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
    assert "smoke_phase1_isaac.sh --gui" in text
    assert "Acceptance #7" in text


def test_spec_and_cursorrules_point_at_run_verification():
    spec = (REPO / "spec.md").read_text(encoding="utf-8")
    rules = (REPO / ".cursorrules").read_text(encoding="utf-8")
    assert "run_verification.sh" in spec
    assert "run_verification.sh" in rules
    assert "./scripts/run_verification.sh ci" in spec
    assert "./scripts/run_verification.sh spark" in spec
