# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Tests for the end-of-test SKIPPED_UNREACHABLE analyzer (spec.md Phase 2)."""
from __future__ import annotations

from residual_adaptive_ik.planning.skipped_analysis import (
    SkippedEpisode,
    analyze_and_report,
    parse_skipped_unreachable,
    render_analysis_markdown,
    speculate_reason,
    speculate_via_for_deterministic_ik,
)

_LOG = """
[3/48] EPISODE trial=2 ik=OK
  DEXTERITY_PRESCREEN: contact pose infeasible (unreachable_orientation; ...)
[3/48] RESULT SKIPPED_UNREACHABLE reason=unreachable_orientation in_region=1 radial_m=0.235 pos_err_m=0.0021 ori_err_rad=0.640 orientations=18 axis=none target=(0.036,-0.237,0.080) | STATUS ok=2 fail=0
[3/48] RESULT SKIPPED_UNREACHABLE reason=unreachable_orientation in_region=1 radial_m=0.235 pos_err_m=0.0021 ori_err_rad=0.640 orientations=18 axis=none target=(0.036,-0.237,0.080) | STATUS ok=2 fail=0
[7/48] RESULT SKIPPED_UNREACHABLE reason=unreachable_position in_region=0 radial_m=0.470 pos_err_m=0.1900 ori_err_rad=1.200 orientations=18 axis=none target=(0.300,0.300,0.150) | STATUS ok=5 fail=1
[8/48] RESULT SKIPPED_UNREACHABLE reason=outside_dexterous_region in_region=0 radial_m=0.276 pos_err_m=nan ori_err_rad=nan orientations=0 axis=none backend=pinocchio target=(0.114,-0.200,0.152) | STATUS ok=5 fail=1
"""


def test_parse_dedupes_double_written_lines() -> None:
    eps = parse_skipped_unreachable(_LOG)
    # The orientation-limited line is duplicated in the log; parser dedupes it.
    assert len(eps) == 3
    assert eps[0].reason == "unreachable_orientation"
    assert eps[0].in_region is True
    assert eps[0].target_m == (0.036, -0.237, 0.080)
    assert eps[1].reason == "unreachable_position"
    assert eps[1].in_region is False
    assert eps[2].reason == "outside_dexterous_region"
    assert eps[2].pos_err_m != eps[2].pos_err_m  # nan


def test_speculate_reason_distinguishes_cases() -> None:
    orient = SkippedEpisode("unreachable_orientation", True, 0.235, 0.002, 0.64, 18, "none")
    pos = SkippedEpisode("unreachable_position", False, 0.47, 0.19, 1.2, 18, "none")
    out = SkippedEpisode("outside_dexterous_region", False, 0.26, float("nan"), float("nan"), 0, "none")
    assert "ORIENTATION" in speculate_reason(orient)
    assert "workspace-edge" in speculate_reason(pos)
    assert "Dexterous" in speculate_reason(out)


def test_via_speculation_only_for_in_region() -> None:
    orient = SkippedEpisode("unreachable_orientation", True, 0.235, 0.002, 0.64, 18, "none")
    pos = SkippedEpisode("unreachable_position", False, 0.47, 0.19, 1.2, 18, "none")
    via = speculate_via_for_deterministic_ik(orient)
    assert via is not None and "deterministic" in via.lower()
    assert speculate_via_for_deterministic_ik(pos) is None


def test_render_markdown_contains_sections() -> None:
    eps = parse_skipped_unreachable(_LOG)
    md = render_analysis_markdown(eps, timestamp="2026-07-17 08:00 -0700", total_planned=39)
    assert "SKIPPED_UNREACHABLE analysis" in md
    assert "within the Dexterous Region" in md
    assert "Deterministic-IK via" in md
    assert "39 target(s) were planned" in md


def test_render_markdown_handles_empty() -> None:
    md = render_analysis_markdown([], timestamp="2026-07-17 08:00 -0700")
    assert "No `SKIPPED_UNREACHABLE`" in md


def test_analyze_and_report_appends_to_status(tmp_path) -> None:
    status = tmp_path / "STATUS.md"
    status.write_text("# STATUS\n\nexisting content\n", encoding="utf-8")
    printed: list[str] = []
    md = analyze_and_report(
        _LOG,
        status_path=status,
        total_planned=39,
        timestamp="2026-07-17 08:00 -0700",
        print_fn=printed.append,
    )
    after = status.read_text(encoding="utf-8")
    assert "existing content" in after  # never overwrites prior content
    assert "SKIPPED_UNREACHABLE analysis" in after
    assert md in after
    assert printed and "SKIPPED_UNREACHABLE analysis" in printed[0]
