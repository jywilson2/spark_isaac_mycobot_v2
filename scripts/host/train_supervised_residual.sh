#!/usr/bin/env bash
# Train Phase 3 supervised residual MLP on the Isaac Sim host python (has torch).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_require_native_shell || true
spark_host_apply_env || exit 1

cd "${SPARK_REPO_ROOT}"
export PYTHONPATH="${SPARK_REPO_ROOT}/src:${PYTHONPATH:-}"
EPOCHS="${1:-50}"
echo "=== Phase 3 supervised residual training (epochs=${EPOCHS}) ==="
"${ISAACSIM_PYTHON_EXE}" -c "import torch; print('torch', torch.__version__)"
"${ISAACSIM_PYTHON_EXE}" -m residual_adaptive_ik.data.generate_supervised_data \
  --train 800 --val 200 --test 200 --stress 200 --seed 42
"${ISAACSIM_PYTHON_EXE}" -m residual_adaptive_ik.learning.train_supervised --epochs "${EPOCHS}"
"${ISAACSIM_PYTHON_EXE}" -m residual_adaptive_ik.learning.evaluate_supervised
echo "=== Phase 3 supervised training complete ==="
