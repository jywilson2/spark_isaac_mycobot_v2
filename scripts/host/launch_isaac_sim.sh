#!/usr/bin/env bash
# Launch the Isaac Sim GUI from the host (DGX Spark native shell).
#
# Usage:
#   ./scripts/host/launch_isaac_sim.sh
#   ./scripts/host/launch_isaac_sim.sh -- /path/to/scene.usd
#
# Resolves ISAACSIM_PATH via scripts/isaac_sim_env.sh (override if needed):
#   export ISAACSIM_PATH="$HOME/isaacsim"
#   export ISAACSIM_PATH="$HOME/IsaacSim/_build/linux-aarch64/release"
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"

if [[ -f /.dockerenv && "${SPARK_ALLOW_CONTAINER_ISAAC:-0}" != "1" ]]; then
  echo "Launch Isaac Sim from a DGX Spark **host** shell (not this container)." >&2
  echo "  ${SPARK_REPO_ROOT}/scripts/host/launch_isaac_sim.sh" >&2
  echo "(Do not use isaac-ros activate for this — that enters the ROS container.)" >&2
  exit 2
fi

require_isaac_python || exit 1

ISAAC_SIM_SH="${ISAACSIM_PATH}/isaac-sim.sh"
if [[ ! -x "${ISAAC_SIM_SH}" ]]; then
  # Some installs name the launcher differently
  for cand in "${ISAACSIM_PATH}/isaac-sim.sh" "${ISAACSIM_PATH}/isaacsim.sh" "${ISAACSIM_PATH}/omni.app.full.sh"; do
    if [[ -x "${cand}" ]]; then
      ISAAC_SIM_SH="${cand}"
      break
    fi
  done
fi

if [[ ! -x "${ISAAC_SIM_SH}" ]]; then
  echo "Could not find isaac-sim.sh under ${ISAACSIM_PATH}" >&2
  echo "Falling back to python.sh Kit bootstrap (empty stage)." >&2
  echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
  echo "ISAACSIM_PYTHON_EXE=${ISAACSIM_PYTHON_EXE}"
  exec "${ISAACSIM_PYTHON_EXE}" -c "from isaacsim import SimulationApp; app=SimulationApp({'headless': False});
import time
print('Isaac Sim GUI running — close the window to exit.')
while app.is_running():
    app.update()
app.close()"
fi

echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
echo "Launching: ${ISAAC_SIM_SH} $*"
exec "${ISAAC_SIM_SH}" "$@"
