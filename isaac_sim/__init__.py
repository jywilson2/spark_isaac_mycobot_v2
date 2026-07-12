# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Isaac Sim helpers for Phase 1 visualization (host GPU only).

Classical IK metrics still run via NumPy; this package drives the MyCobot USD
in Isaac Sim so you can *see* FK/IK. See ``spec.md`` Phase 1 and
``scripts/host/run_isaac_viz.sh``.
"""
from __future__ import annotations
