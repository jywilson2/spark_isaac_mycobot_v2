# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""End-of-test analysis of ``SKIPPED_UNREACHABLE`` episodes.

Requirement (spec.md Phase 2 — end-of-test SKIPPED_UNREACHABLE analysis)
------------------------------------------------------------------------
At the end of each viz/smoke test, scan the run log for
``RESULT SKIPPED_UNREACHABLE`` lines emitted by the dexterity prescreen
(``planning/dexterity.py``). For each one:

1. **Speculate why** the target was skipped (raw reach limit vs orientation
   feasibility), from the metrics recorded on the line.
2. If the target is **still within the "Dexterous Region"** (its position is
   comfortably reachable — ``in_region=1`` or ``reason=unreachable_orientation``),
   **speculate a different via** that would support a *deterministic* IK
   calculation (e.g. seed cuRobo from the classical DLS/analytic solution at a
   well-conditioned pre-approach).

The rendered analysis is returned for printing in the prompt output **and**
appended to ``STATUS.md`` (see ``analyze_and_report``).

Pure + deterministic (no Isaac/GPU); unit-tested from log fixtures.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

_NUM = r"(?:[-\d.]+|nan|NaN|inf|-inf)"
_SKIP_RE = re.compile(
    r"RESULT\s+SKIPPED_UNREACHABLE\s+"
    r"reason=(?P<reason>\w+)\s+"
    r"in_region=(?P<in_region>[01])\s+"
    r"radial_m=(?P<radial>" + _NUM + r")\s+"
    r"pos_err_m=(?P<pos_err>" + _NUM + r")\s+"
    r"ori_err_rad=(?P<ori_err>" + _NUM + r")\s+"
    r"orientations=(?P<orientations>\d+)\s+"
    r"axis=(?P<axis>\w+)"
    r"(?:\s+backend=(?P<backend>\w+))?"
    r"(?:.*?target=\((?P<tx>[-\d.]+),(?P<ty>[-\d.]+),(?P<tz>[-\d.]+)\))?"
)


@dataclass
class SkippedEpisode:
    """One parsed ``SKIPPED_UNREACHABLE`` record (SI units)."""

    reason: str
    in_region: bool
    radial_m: float
    pos_err_m: float
    ori_err_rad: float
    orientations: int
    axis: str
    target_m: tuple[float, float, float] | None = None


def parse_skipped_unreachable(log_text: str) -> list[SkippedEpisode]:
    """Extract all ``SKIPPED_UNREACHABLE`` records from a run log (deduped)."""
    seen: set[tuple] = set()
    episodes: list[SkippedEpisode] = []
    for m in _SKIP_RE.finditer(str(log_text or "")):
        target = None
        if m.group("tx") is not None:
            target = (
                float(m.group("tx")),
                float(m.group("ty")),
                float(m.group("tz")),
            )
        key = (m.group("reason"), round(float(m.group("radial")), 4), target)
        if key in seen:  # the log double-writes (timestamped + plain)
            continue
        seen.add(key)
        episodes.append(
            SkippedEpisode(
                reason=m.group("reason"),
                in_region=bool(int(m.group("in_region"))),
                radial_m=float(m.group("radial")),
                pos_err_m=float(m.group("pos_err")),
                ori_err_rad=float(m.group("ori_err")),
                orientations=int(m.group("orientations")),
                axis=m.group("axis"),
                target_m=target,
            )
        )
    return episodes


def speculate_reason(ep: SkippedEpisode) -> str:
    """Human speculation for *why* this episode was skipped."""
    if ep.reason == "unreachable_orientation" or (
        ep.in_region and ep.reason != "outside_dexterous_region"
    ):
        return (
            f"Position is comfortably reachable (radial {ep.radial_m:.3f} m, inside "
            "the Dexterous Region), but no pad-facing orientation within the "
            f"contact cone solved IK (best orientation error {ep.ori_err_rad:.2f} "
            "rad). Most likely an ORIENTATION-feasibility limit: the wrist cannot "
            "attain the outward-normal tool axis at this pose (near a wrist "
            "singularity / joint-limit for that approach azimuth)."
        )
    if ep.reason == "outside_dexterous_region":
        return (
            f"Target radial {ep.radial_m:.3f} m is outside the geometric Dexterous "
            "Region (interior reach shell). Near-envelope targets routinely "
            "IK_FAIL the oriented contact plan in cuRobo even when a collision-free "
            "joint solution exists in isolation — correctly excluded from the "
            "PLAN_OK gate as sampler optimism, not a planner fault."
        )
    return (
        f"Target radial {ep.radial_m:.3f} m is outside the Dexterous-Region "
        "interior shell (near/beyond the reach envelope). The pad-facing contact "
        f"pose is unreachable at any wrist orientation (best position error "
        f"{ep.pos_err_m:.3f} m). This is a genuine workspace-edge target, "
        "correctly excluded from the PLAN_OK gate rather than counted as a "
        "planner failure."
    )


def speculate_via_for_deterministic_ik(ep: SkippedEpisode) -> str | None:
    """If in the Dexterous Region, propose a via for a *deterministic* IK.

    Returns ``None`` for genuine out-of-reach targets (a via cannot add reach).
    """
    if not (ep.reason == "unreachable_orientation" or ep.in_region):
        return None
    clearance = max(0.06, min(0.16, ep.radial_m * 0.5))
    return (
        "Because the position IS reachable, prefer a two-stage deterministic "
        "approach instead of relying on cuRobo's stochastic IK seeds: "
        f"(1) plan a pre-approach via to a well-conditioned wrist pose on the "
        f"base→target radial at ~{clearance:.2f} m clearance, solved by the "
        "classical DLS IK (deterministic) — or an analytic/TRAC-IK seed; "
        "(2) from there, seed the cuRobo contact plan with that DLS joint "
        "solution and sweep the approach azimuth (roll about the outward normal) "
        "within the bounded orientation cone. Seeding the optimizer from a "
        "deterministic classical solution removes the run-to-run IK_FAIL "
        "flakiness for this in-region target. If still infeasible across the "
        "whole azimuth sweep, treat the pad-facing constraint itself as "
        "over-constrained here and relax the tool-axis cone (still honest, "
        "≤ gate tol) for this envelope band."
    )


def render_analysis_markdown(
    episodes: list[SkippedEpisode],
    *,
    timestamp: str | None = None,
    total_planned: int | None = None,
) -> str:
    """Render the SKIPPED_UNREACHABLE analysis as a Markdown block."""
    ts = timestamp or datetime.now().astimezone().strftime("%Y-%m-%d %H:%M %z")
    lines: list[str] = []
    lines.append(f"## SKIPPED_UNREACHABLE analysis (auto, {ts})")
    lines.append("")
    if not episodes:
        lines.append(
            "No `SKIPPED_UNREACHABLE` episodes in this run — the dexterity "
            "prescreen skipped nothing (all planned targets were orientation-"
            "feasible, or the prescreen was disabled)."
        )
        lines.append("")
        return "\n".join(lines)
    n_region = sum(1 for e in episodes if (e.in_region or e.reason == "unreachable_orientation"))
    lines.append(
        f"{len(episodes)} target(s) skipped as unreachable; **{n_region} within "
        "the Dexterous Region** (orientation-limited, via speculation below), "
        f"{len(episodes) - n_region} genuine workspace-edge."
    )
    if total_planned is not None:
        lines.append("")
        lines.append(
            f"(These are excluded from the PLAN_OK gate denominator; "
            f"{total_planned} target(s) were planned.)"
        )
    lines.append("")
    for i, ep in enumerate(episodes, 1):
        tgt = (
            f"({ep.target_m[0]:.3f}, {ep.target_m[1]:.3f}, {ep.target_m[2]:.3f})"
            if ep.target_m is not None
            else "(unknown)"
        )
        lines.append(f"### Skipped #{i} — target {tgt} (radial {ep.radial_m:.3f} m)")
        lines.append("")
        lines.append(f"- **Classification:** `{ep.reason}` (in_region={int(ep.in_region)})")
        lines.append(f"- **Why (speculation):** {speculate_reason(ep)}")
        via = speculate_via_for_deterministic_ik(ep)
        if via is not None:
            lines.append(f"- **Deterministic-IK via (speculation):** {via}")
        else:
            lines.append(
                "- **Deterministic-IK via:** none — target is beyond the "
                "dexterous reach; a via cannot add reach. Exclude honestly."
            )
        lines.append("")
    return "\n".join(lines)


def analyze_and_report(
    log_text: str,
    *,
    status_path: Path | str | None = None,
    total_planned: int | None = None,
    timestamp: str | None = None,
    print_fn=print,
) -> str:
    """Parse, render, print, and append the analysis to ``STATUS.md``.

    Returns the rendered Markdown (also printed via ``print_fn`` for the prompt
    output). When ``status_path`` is given, the block is **appended** (never
    overwriting prior content, per the documentation policy).
    """
    episodes = parse_skipped_unreachable(log_text)
    md = render_analysis_markdown(
        episodes, timestamp=timestamp, total_planned=total_planned
    )
    if print_fn is not None:
        print_fn(md)
    if status_path is not None:
        p = Path(status_path)
        try:
            existing = p.read_text(encoding="utf-8") if p.exists() else ""
            p.write_text(existing + "\n\n" + md + "\n", encoding="utf-8")
        except OSError as exc:
            # Common on Spark: STATUS.md owned by root while Kit runs as the
            # host user — silent pass hid missing analysis appends.
            if print_fn is not None:
                print_fn(f"WARNING: could not append SKIPPED_UNREACHABLE analysis to {p}: {exc}")
    return md
