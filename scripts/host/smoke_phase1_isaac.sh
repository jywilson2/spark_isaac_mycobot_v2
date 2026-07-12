#!/usr/bin/env bash
# Host Phase 1 Isaac Sim smoke (TDD / verification).
#
# Runs from a **native host shell** (or via nsenter host mount) with Kit installed.
#
# Verification policy (spec.md Acceptance #7):
#   - CI / remote GitHub PR: headless only (this script's default).
#   - DGX Spark host with Isaac Sim: after headless succeeds, required --gui
#     from a native desktop session (DISPLAY). Prefer not nsenter-as-root for GUI.
#
#   ./scripts/host/smoke_phase1_isaac.sh           # headless (CI default)
#   ./scripts/host/smoke_phase1_isaac.sh --gui     # visible Isaac Sim window
#
# GUI requires a host graphical session with DISPLAY (e.g. desktop login).
# Prefer running --gui from a native host terminal, not only via nsenter.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"

GUI=0
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)
      GUI=1
      shift
      ;;
    --)
      shift
      EXTRA+=("$@")
      break
      ;;
    *)
      EXTRA+=("$1")
      shift
      ;;
  esac
done

if [[ -f /.dockerenv && "${SPARK_ALLOW_CONTAINER_ISAAC:-0}" != "1" ]]; then
  echo "This smoke must run on the host (or with SPARK_ALLOW_CONTAINER_ISAAC=1)." >&2
  echo "From the container you can try:" >&2
  echo "  ./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh" >&2
  echo "For a visible GUI, use a native host terminal:" >&2
  echo "  ./scripts/host/smoke_phase1_isaac.sh --gui" >&2
  exit 2
fi

spark_host_require_native_shell || true
spark_host_apply_env || exit 1

"${SPARK_REPO_ROOT}/scripts/download_mycobot_ros2.sh"

N_POSES="${PHASE1_SMOKE_N_POSES:-240}"
N_VIZ="${PHASE1_SMOKE_VISUALIZE:-48}"
HOLD="${PHASE1_SMOKE_HOLD_S:-0.2}"

ARGS=(--skip-tests -- --num-poses "${N_POSES}" --visualize "${N_VIZ}" --hold-s "${HOLD}")
if [[ "${GUI}" -eq 0 ]]; then
  ARGS+=(--headless)
  echo "NOTE: running headless (no GUI). Pose animation still runs in-Kit."
  echo "      For a visible window: $0 --gui   (host desktop session with DISPLAY)"
else
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: --gui requested but DISPLAY is unset." >&2
    echo "Run from a graphical host login, or export DISPLAY (e.g. :1)." >&2
    exit 1
  fi
  echo "NOTE: GUI mode (DISPLAY=${DISPLAY}). Isaac Sim window should appear."
  # Auto-close after animation so agents / nsenter can finish without manual Ctrl+C.
  # Interactive keep-open: PHASE1_SMOKE_KEEP_GUI_OPEN=1 ./scripts/host/smoke_phase1_isaac.sh --gui
  if [[ "${PHASE1_SMOKE_KEEP_GUI_OPEN:-0}" != "1" ]]; then
    ARGS+=(--auto-exit)
  fi
fi
ARGS+=("${EXTRA[@]+"${EXTRA[@]}"}")

echo "=== Phase 1 Isaac smoke (host) ==="
echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
echo "GUI=${GUI} n_poses=${N_POSES} visualize=${N_VIZ} (visualize=animate trials, not 'show window')"

# Force re-import when fixing drive gains so USD is regenerated
if [[ "${PHASE1_SMOKE_KEEP_PREPARED:-0}" == "1" ]]; then
  ARGS+=(--keep-prepared)
fi

"${SCRIPT_DIR}/run_phase1_isaac.sh" "${ARGS[@]}"
echo "Phase 1 Isaac smoke PASSED"
