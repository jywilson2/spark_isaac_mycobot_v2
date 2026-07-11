#!/usr/bin/env bash
# Phase 2 supervised residual training entry (stub until implemented).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
echo "Phase 2: running supervised training entry (expects implementation)."
python -m residual_adaptive_ik.learning.train_supervised \
  --config "${ROOT}/configs/learning/supervised_residual.yaml" "$@"
