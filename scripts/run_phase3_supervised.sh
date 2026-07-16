#!/usr/bin/env bash
# Phase 3 supervised residual: generate datasets, train (host torch), evaluate.
# Prefer host Isaac python via spark_host_exec when in the Cursor/Isaac ROS container.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

EPOCHS="${1:-100}"

if [[ -f /.dockerenv ]] && [[ -f "${ROOT}/scripts/host/spark_host_exec.sh" ]]; then
  echo "Phase 3: delegating train+eval to Isaac Sim host (torch)."
  exec bash "${ROOT}/scripts/host/spark_host_exec.sh" \
    ./scripts/host/train_supervised_residual.sh "${EPOCHS}"
fi

export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

echo "Phase 3: generate datasets + train + evaluate (local python)."
python3 -m residual_adaptive_ik.data.generate_supervised_data \
  --train 800 --val 200 --test 200 --stress 200 --seed 42
python3 -m residual_adaptive_ik.learning.train_supervised --epochs "${EPOCHS}"
python3 -m residual_adaptive_ik.learning.evaluate_supervised
echo "See docs/phase3_supervised.md"
