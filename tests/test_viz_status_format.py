# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Contracts for GUI-test log formatting and via-waypoint warnings.

Why: operators monitor the Isaac Sim GUI smoke in real time. Every episode
must end with a grep-friendly ``RESULT ... | STATUS ok=.. fail=.. via=..``
line, and targets that were unreachable by a direct plan (needed intermediate
standoff waypoints) must raise a **warning** that is mirrored into the Kit
Console (``carb.log_warn``) — see ``_viz_log`` in ``isaac_sim/run_ik_viz.py``
and spec.md § Sequential multi-target sequences.
"""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
VIZ_SRC = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")


def test_via_waypoint_warning_in_gui():
    """Unreachable-direct targets must warn in the Isaac Sim GUI console."""
    assert "VIA_WAYPOINT_USED" in VIZ_SRC
    assert 'used_via = (' in VIZ_SRC or "strategy=via_contact" in VIZ_SRC
    assert "strategy=via_standoff" in VIZ_SRC
    assert "strategy=via_contact" in VIZ_SRC
    # Success messages carry via_attempts=N (not via1_ markers) — parse that.
    assert r"via_attempts=(\d+)" in VIZ_SRC
    # The warning must use the warn level so carb.log_warn reaches the Kit UI.
    idx = VIZ_SRC.index("VIA_WAYPOINT_USED")
    assert 'level="warn"' in VIZ_SRC[idx : idx + 400]


def test_every_episode_ends_with_result_and_tally():
    """RESULT lines cover all episode outcomes and embed the running tally."""
    assert "RESULT IK_FAIL" in VIZ_SRC
    assert "RESULT SKIPPED" in VIZ_SRC
    assert "RESULT PLAN_OK" in VIZ_SRC
    assert "RESULT PLAN_FAIL" in VIZ_SRC
    assert "RESULT PLAN_FAIL(no_contact)" in VIZ_SRC
    assert "def _tally()" in VIZ_SRC
    assert "STATUS ok=" in VIZ_SRC


def test_summary_and_metrics_include_via_and_skip_counts():
    assert "phase2_via_waypoint_ok" in VIZ_SRC
    assert "phase2_skipped_targets" in VIZ_SRC
    assert "via={n_via_ok} skip={n_skip}" in VIZ_SRC


def test_summary_warns_when_issues_present():
    """Final summary escalates to warn when any fail or via recovery occurred."""
    assert 'level="warn" if (n_plan_fail > 0 or n_via_ok > 0) else "info"' in VIZ_SRC
