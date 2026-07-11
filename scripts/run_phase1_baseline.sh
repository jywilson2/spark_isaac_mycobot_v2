#!/usr/bin/env bash
# Phase 1 baseline: run FK/IK unit tests and remind about metrics report.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export SPARK_REPO_ROOT="${ROOT}"
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
pytest tests/test_fk.py tests/test_ik_validation.py tests/test_residual_bounds.py -q
echo
echo "FK is implemented (URDF). Numerical IK + validation still need implementation."
echo "When complete, write metrics to docs/phase1_baseline.md (≥1000 reachable poses)."
