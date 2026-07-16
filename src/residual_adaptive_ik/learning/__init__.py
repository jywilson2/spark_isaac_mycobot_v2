# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 3–4 residual learning (supervised MLP, then SAC).

Deployed path remains ``q_final = q_ik + clamp(Δq)``. Training never commands
physical hardware. See ``spec.md`` Phases 3–4.
"""
from __future__ import annotations

