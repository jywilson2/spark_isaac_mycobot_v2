#!/usr/bin/env bash
# Phase 1 baseline: unit tests + ≥1000-pose DLS IK metrics report.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export SPARK_REPO_ROOT="${ROOT}"
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

N_POSES="${PHASE1_N_POSES:-1000}"
SEED="${PHASE1_SEED:-0}"

pytest tests/test_fk.py tests/test_ik_validation.py tests/test_residual_bounds.py -q
python -m residual_adaptive_ik.kinematics.baseline_eval \
  --n-poses "${N_POSES}" \
  --seed "${SEED}" \
  --json-out "${ROOT}/assets/logs/phase1_baseline_metrics.json" \
  --md-out "${ROOT}/docs/phase1_baseline.md"

echo
echo "Phase 1 baseline complete. See docs/phase1_baseline.md"
