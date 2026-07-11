#!/usr/bin/env bash
# Convert MyCobot URDF to USD using host Isaac Sim (delegates via spark_host_exec).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=host/spark_host_exec.sh
source "${ROOT}/scripts/host/spark_host_exec.sh"

echo "URDF→USD conversion requires Isaac Sim on the host."
echo "Wire convert_urdf_to_usd.sh to your Isaac Sim importer once Phase 1 URDF path is fixed."
echo "Hint URDF: configs/robot/mycobot_280.yaml → urdf_hint"
exit 1
