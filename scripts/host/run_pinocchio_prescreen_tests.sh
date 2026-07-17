#!/usr/bin/env bash
# Run Pinocchio dexterity-gate unit tests under Isaac Sim python (host).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
cd "${SPARK_REPO_ROOT}"
export PYTHONPATH="${SPARK_REPO_ROOT}/src:${SPARK_REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export SPARK_RUN_ISAAC_SMOKE=0
export SPARK_RUN_ISAAC_GUI_SMOKE=0
"${ISAACSIM_PATH}/python.sh" -m pytest \
  tests/test_pinocchio_ik.py \
  tests/test_dexterity.py \
  -q
