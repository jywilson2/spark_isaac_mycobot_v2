#!/usr/bin/env bash
# Verify Isaac Lab is installed on the **host** (Phase 3 prerequisite).
#
# Usage (host terminal after isaac-ros activate / native shell):
#   ./scripts/host/verify_isaac_lab.sh
#
# Inside the Isaac ROS / Cursor container this script exits early unless you
# set SPARK_ALLOW_CONTAINER_ISAAC=1 and point ISAACSIM_PATH / ISAACLAB_PATH at
# a mounted host install.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
# shellcheck source=../../isaac_lab/versions.env
source "${REPO_ROOT}/isaac_lab/versions.env"

if [[ -f /.dockerenv && "${SPARK_ALLOW_CONTAINER_ISAAC:-0}" != "1" ]]; then
  echo "Isaac Lab verification is intended for the DGX Spark **host** shell." >&2
  echo "  isaac-ros activate   # host" >&2
  echo "  ${REPO_ROOT}/scripts/host/verify_isaac_lab.sh" >&2
  echo "Or mount Isaac and set SPARK_ALLOW_CONTAINER_ISAAC=1 ISAACSIM_PATH=... ISAACLAB_PATH=..." >&2
  exit 2
fi

spark_host_apply_env || {
  echo "Isaac Sim python.sh not found. Set ISAACSIM_PATH or ISAACSIM_PYTHON_EXE." >&2
  exit 1
}

export ISAACLAB_PATH="${ISAACLAB_PATH:-${SPARK_ISAACLAB_PATH}}"
export SPARK_REPO_ROOT="${REPO_ROOT}"

if [[ ! -x "${ISAACLAB_PATH}/isaaclab.sh" ]]; then
  echo "Isaac Lab not installed at ${ISAACLAB_PATH}." >&2
  echo "Run: ${REPO_ROOT}/scripts/host/install_isaac_lab.sh" >&2
  exit 1
fi

echo "=== Host Isaac Lab detect ==="
(
  cd "${ISAACLAB_PATH}"
  ./isaaclab.sh -p "${REPO_ROOT}/isaac_lab/detect_isaac_lab.py"
)

echo "Isaac Lab detect complete."
echo "Note: Phase 3 residual env / SAC training scripts are not wired yet (see STATUS.md)."
