# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Even Cartesian workspace sampling for Phase 1 IK targets (meters).

Mirrors v1 ``isaac_lab/mdp_core.py`` cylindrical reach envelope and stratified
demo bins so IK goals cover the arm's working volume instead of clustering from
raw joint-space sampling. See ``spec.md`` Phase 1 and v1 ``MAX_BLOCK_REACH_M``.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_WORKSPACE_PATH = _REPO_ROOT / "configs" / "robot" / "workspace.yaml"


@dataclass(frozen=True)
class WorkspaceEnvelope:
    """Cylindrical reach envelope in the robot base frame (SI meters)."""

    min_reach_m: float
    max_reach_m: float
    min_z_m: float
    max_z_m: float
    azimuth_bins: int
    radius_bins: int
    z_bins: int
    max_joint_speed_deg_s: float

    @property
    def max_joint_speed_rad_s(self) -> float:
        return float(np.deg2rad(self.max_joint_speed_deg_s))

    @property
    def n_bins(self) -> int:
        return int(self.azimuth_bins * self.radius_bins * self.z_bins)


def load_workspace_config(path: Path | None = None) -> WorkspaceEnvelope:
    """Load workspace envelope from ``configs/robot/workspace.yaml``."""
    cfg_path = Path(path) if path is not None else DEFAULT_WORKSPACE_PATH
    with cfg_path.open(encoding="utf-8") as handle:
        raw: dict[str, Any] = dict(yaml.safe_load(handle) or {})
    return WorkspaceEnvelope(
        min_reach_m=float(raw["min_reach_m"]),
        max_reach_m=float(raw["max_reach_m"]),
        min_z_m=float(raw["min_z_m"]),
        max_z_m=float(raw["max_z_m"]),
        azimuth_bins=int(raw.get("azimuth_bins", 8)),
        radius_bins=int(raw.get("radius_bins", 3)),
        z_bins=int(raw.get("z_bins", 4)),
        max_joint_speed_deg_s=float(raw.get("max_joint_speed_deg_s", 160.0)),
    )


def sample_point_in_bin(
    rng: np.random.Generator,
    envelope: WorkspaceEnvelope,
    *,
    azimuth_bin: int,
    radius_bin: int,
    z_bin: int,
) -> np.ndarray:
    """Sample one XYZ (m) inside a stratified cell (jitter in bin interior)."""
    az_n = envelope.azimuth_bins
    r_n = envelope.radius_bins
    z_n = envelope.z_bins
    # Match v1: avoid bin edges with interior jitter [0.15, 0.85)
    angle = (azimuth_bin + float(rng.uniform(0.15, 0.85))) / az_n * (2.0 * np.pi)
    r_span = (envelope.max_reach_m - envelope.min_reach_m) / r_n
    # Area-ish: uniform in radius within the bin (v1 demo cell); bins themselves
    # partition the annulus for even coverage across cells.
    radius = float(
        rng.uniform(
            envelope.min_reach_m + radius_bin * r_span,
            envelope.min_reach_m + (radius_bin + 1) * r_span,
        )
    )
    z_span = (envelope.max_z_m - envelope.min_z_m) / z_n
    z = float(
        rng.uniform(
            envelope.min_z_m + z_bin * z_span,
            envelope.min_z_m + (z_bin + 1) * z_span,
        )
    )
    return np.array([radius * np.cos(angle), radius * np.sin(angle), z], dtype=float)


def sample_stratified_workspace_positions(
    n: int,
    rng: np.random.Generator,
    envelope: WorkspaceEnvelope | None = None,
) -> np.ndarray:
    """Return ``(n, 3)`` XYZ samples with even bin coverage (round-robin cells).

    Cycles through all azimuth×radius×z bins so large ``n`` fills the workspace
    evenly (v1 demo stratified requirement). Units: meters.
    """
    if n <= 0:
        return np.zeros((0, 3), dtype=float)
    env = envelope or load_workspace_config()
    bins = [
        (az, r, z)
        for az in range(env.azimuth_bins)
        for r in range(env.radius_bins)
        for z in range(env.z_bins)
    ]
    order = np.arange(len(bins))
    rng.shuffle(order)
    bins = [bins[i] for i in order]
    positions = np.zeros((n, 3), dtype=float)
    for i in range(n):
        az_b, r_b, z_b = bins[i % len(bins)]
        positions[i] = sample_point_in_bin(
            rng, env, azimuth_bin=az_b, radius_bin=r_b, z_bin=z_b
        )
    return positions


def assign_workspace_bin(
    position_m: np.ndarray,
    envelope: WorkspaceEnvelope,
) -> tuple[int, int, int] | None:
    """Map a point to ``(azimuth_bin, radius_bin, z_bin)`` or ``None`` if outside."""
    p = np.asarray(position_m, dtype=float).reshape(3)
    radius = float(np.hypot(p[0], p[1]))
    if radius < envelope.min_reach_m or radius > envelope.max_reach_m:
        return None
    if p[2] < envelope.min_z_m or p[2] > envelope.max_z_m:
        return None
    angle = float(np.arctan2(p[1], p[0]))
    if angle < 0.0:
        angle += 2.0 * np.pi
    az_bin = min(
        envelope.azimuth_bins - 1,
        int(angle / (2.0 * np.pi) * envelope.azimuth_bins),
    )
    r_frac = (radius - envelope.min_reach_m) / (
        envelope.max_reach_m - envelope.min_reach_m + 1e-15
    )
    r_bin = min(envelope.radius_bins - 1, int(r_frac * envelope.radius_bins))
    z_frac = (p[2] - envelope.min_z_m) / (envelope.max_z_m - envelope.min_z_m + 1e-15)
    z_bin = min(envelope.z_bins - 1, int(z_frac * envelope.z_bins))
    return az_bin, r_bin, z_bin


def bin_occupancy_counts(
    positions_m: np.ndarray,
    envelope: WorkspaceEnvelope,
) -> np.ndarray:
    """Return length-``n_bins`` occupancy counts for stratified coverage tests."""
    counts = np.zeros(envelope.n_bins, dtype=int)
    for p in np.asarray(positions_m, dtype=float).reshape(-1, 3):
        assigned = assign_workspace_bin(p, envelope)
        if assigned is None:
            continue
        az_b, r_b, z_b = assigned
        idx = (
            az_b * envelope.radius_bins * envelope.z_bins
            + r_b * envelope.z_bins
            + z_b
        )
        counts[idx] += 1
    return counts
