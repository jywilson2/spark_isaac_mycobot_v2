#!/usr/bin/env bash
# Fast headless contact-stack iteration (no Kit window, time-warp, early abort).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOG="${1:-/tmp/headless_contact_iter.log}"

export ISAAC_VIZ_SMOKE_N_POSES="${ISAAC_VIZ_SMOKE_N_POSES:-40}"
export ISAAC_VIZ_SMOKE_VISUALIZE="${ISAAC_VIZ_SMOKE_VISUALIZE:-8}"
export ISAAC_VIZ_SMOKE_HOLD_S="${ISAAC_VIZ_SMOKE_HOLD_S:-0.15}"
export ISAAC_VIZ_SMOKE_TIME_WARP="${ISAAC_VIZ_SMOKE_TIME_WARP:-4}"
export ISAAC_VIZ_SMOKE_RESET_TO_HOME="${ISAAC_VIZ_SMOKE_RESET_TO_HOME:-0}"
export ISAAC_VIZ_SHOW_COLLISION_SPHERES="${ISAAC_VIZ_SHOW_COLLISION_SPHERES:-0}"
export ISAAC_VIZ_RECOVERY_TIMEOUT_S="${ISAAC_VIZ_RECOVERY_TIMEOUT_S:-45}"

echo "=== headless contact iter ==="
echo "LOG=${LOG}"
echo "n_poses=${ISAAC_VIZ_SMOKE_N_POSES} visualize=${ISAAC_VIZ_SMOKE_VISUALIZE} warp=${ISAAC_VIZ_SMOKE_TIME_WARP} recovery_s=${ISAAC_VIZ_RECOVERY_TIMEOUT_S}"

cd "${REPO_ROOT}"
exec ./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh \
  -- --early-abort-after-fails 1 --no-show-collision-spheres
