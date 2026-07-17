#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
cd "${SPARK_REPO_ROOT}"
export PYTHONPATH="${SPARK_REPO_ROOT}/src:${SPARK_REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
"${ISAACSIM_PATH}/python.sh" "${SPARK_REPO_ROOT}/scripts/host/smoke_pinocchio_prescreen.py"
