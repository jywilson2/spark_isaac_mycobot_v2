# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 3 supervised residual datasets (schema + generation).

Simulated imperfections (joint bias, tool/base frame, target noise, payload
sag) produce ``ResidualIKSample`` rows with bounded ``Δq`` labels. Classical
IK remains the base; learning only predicts residuals. See ``spec.md`` Phase 3.
"""
from __future__ import annotations

from residual_adaptive_ik.data.dataset_schema import ResidualIKSample
from residual_adaptive_ik.data.generate_supervised_data import (
    OBS_DIM,
    generate_samples,
    load_npz,
    pack_observation,
    write_npz,
)

__all__ = [
    "OBS_DIM",
    "ResidualIKSample",
    "generate_samples",
    "load_npz",
    "pack_observation",
    "write_npz",
]

