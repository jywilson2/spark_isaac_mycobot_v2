#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
cd "${SPARK_REPO_ROOT}"
export PYTHONPATH="${SPARK_REPO_ROOT}/src:${SPARK_REPO_ROOT}"
"${ISAACSIM_PATH}/python.sh" "${SPARK_REPO_ROOT}/scripts/host/probe_pinocchio_gate_rate.py"
