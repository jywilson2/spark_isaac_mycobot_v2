#!/usr/bin/env bash
# Convert MyCobot URDF to USD using host Isaac Sim.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=host/env.isaac_host.sh
source "${ROOT}/scripts/host/env.isaac_host.sh"

if [[ -f /.dockerenv && "${SPARK_ALLOW_CONTAINER_ISAAC:-0}" != "1" ]]; then
  echo "URDF→USD requires Isaac Sim on the **host**." >&2
  echo "  ${ROOT}/scripts/convert_urdf_to_usd.sh" >&2
  echo "Or from container: SPARK_ALLOW_CONTAINER_ISAAC=1 with ISAACSIM_PATH set," >&2
  echo "  or: ./scripts/host/spark_host_exec.sh ./scripts/convert_urdf_to_usd.sh" >&2
  exit 2
fi

spark_host_apply_env || exit 1

if [[ ! -f "${SPARK_REPO_ROOT}/third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf" ]]; then
  "${SPARK_REPO_ROOT}/scripts/download_mycobot_ros2.sh"
fi

LOG_PATH="$(spark_host_new_log urdf_to_usd)"
echo "Writing log: ${LOG_PATH}"
set +e
spark_host_run_python \
  "${SPARK_REPO_ROOT}/isaac_sim/convert_urdf_to_usd.py" \
  --repo-root "${SPARK_REPO_ROOT}" \
  "$@" 2>&1 | tee "${LOG_PATH}"
exit_code="${PIPESTATUS[0]}"
set -e
exit "${exit_code}"
