#!/usr/bin/env bash
# Phase 3 SAC residual training — must run on Isaac Sim host, not in container alone.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=host/spark_host_exec.sh
source "${ROOT}/scripts/host/spark_host_exec.sh"

echo "Phase 3 SAC training uses Isaac Lab on the DGX Spark host."
echo "Container callers should delegate via spark_host_exec (see STATUS.md)."
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
python -m residual_adaptive_ik.learning.train_sac \
  --config "${ROOT}/configs/learning/sac_residual.yaml" "$@"
