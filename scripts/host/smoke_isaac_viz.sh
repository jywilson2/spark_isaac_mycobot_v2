#!/usr/bin/env bash
# Host Isaac Sim IK viz smoke (Phase 1 metrics + Phase 2 planning).
#
# Runs from a **native host shell** (or via nsenter host mount) with Kit installed.
#
# Verification policy (spec.md Acceptance #7):
#   - CI / remote GitHub PR: headless only (this script's default).
#   - DGX Spark host with Isaac Sim: after headless succeeds, required --gui
#     from a native desktop session (DISPLAY). Prefer not nsenter-as-root for GUI.
#
#   ./scripts/host/smoke_isaac_viz.sh           # headless (CI default)
#   ./scripts/host/smoke_isaac_viz.sh --gui     # visible Isaac Sim window
#   ./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
#
# Legacy alias: scripts/host/smoke_isaac_viz.sh → this script.
#
# GUI requires a host graphical session with DISPLAY (e.g. desktop login).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"

GUI=0
RESET_HOME=""
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)
      GUI=1
      shift
      ;;
    --reset-to-home)
      RESET_HOME=1
      shift
      ;;
    --no-reset-to-home)
      RESET_HOME=0
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
  echo "  ./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh" >&2
  echo "For a visible GUI, use a native host terminal:" >&2
  echo "  ./scripts/host/smoke_isaac_viz.sh --gui" >&2
  exit 2
fi

spark_host_require_native_shell || true
spark_host_apply_env || exit 1

"${SPARK_REPO_ROOT}/scripts/download_mycobot_ros2.sh"

# Prefer ISAAC_VIZ_SMOKE_*; accept legacy PHASE1_SMOKE_* aliases.
N_POSES="${ISAAC_VIZ_SMOKE_N_POSES:-${PHASE1_SMOKE_N_POSES:-240}}"
N_VIZ="${ISAAC_VIZ_SMOKE_VISUALIZE:-${PHASE1_SMOKE_VISUALIZE:-48}}"
HOLD="${ISAAC_VIZ_SMOKE_HOLD_S:-${PHASE1_SMOKE_HOLD_S:-0.2}}"
# Fail smoke when PLAN_OK rate is below threshold (YAML default 0.25).
# Set ISAAC_VIZ_MIN_PLAN_OK_RATE=0 to disable (metrics-only / debugging).
MIN_PLAN_OK_RATE="${ISAAC_VIZ_MIN_PLAN_OK_RATE:-}"

ARGS=(--skip-tests -- --num-poses "${N_POSES}" --visualize "${N_VIZ}" --hold-s "${HOLD}")
if [[ -n "${MIN_PLAN_OK_RATE}" ]]; then
  ARGS+=(--min-plan-ok-rate "${MIN_PLAN_OK_RATE}")
fi
# CLI / env override for home reset (YAML default is false).
if [[ -z "${RESET_HOME}" ]]; then
  RESET_HOME="${ISAAC_VIZ_SMOKE_RESET_TO_HOME:-${PHASE1_SMOKE_RESET_TO_HOME:-}}"
fi
if [[ "${RESET_HOME}" == "1" || "${RESET_HOME}" == "true" ]]; then
  ARGS+=(--reset-to-home)
elif [[ "${RESET_HOME}" == "0" || "${RESET_HOME}" == "false" ]]; then
  ARGS+=(--no-reset-to-home)
fi
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
  # Interactive keep-open: ISAAC_VIZ_SMOKE_KEEP_GUI_OPEN=1 ... --gui
  KEEP_OPEN="${ISAAC_VIZ_SMOKE_KEEP_GUI_OPEN:-${PHASE1_SMOKE_KEEP_GUI_OPEN:-0}}"
  if [[ "${KEEP_OPEN}" != "1" ]]; then
    ARGS+=(--auto-exit)
  fi
fi
ARGS+=("${EXTRA[@]+"${EXTRA[@]}"}")

echo "=== Isaac viz smoke (host; Phase 1 metrics + Phase 2 planning) ==="
echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
echo "GUI=${GUI} n_poses=${N_POSES} visualize=${N_VIZ} (visualize=animate trials, not 'show window')"
if [[ -n "${MIN_PLAN_OK_RATE}" ]]; then
  echo "min_plan_ok_rate=${MIN_PLAN_OK_RATE} (ISAAC_VIZ_MIN_PLAN_OK_RATE)"
else
  echo "min_plan_ok_rate=(YAML default; override with ISAAC_VIZ_MIN_PLAN_OK_RATE)"
fi

KEEP_PREP="${ISAAC_VIZ_SMOKE_KEEP_PREPARED:-${PHASE1_SMOKE_KEEP_PREPARED:-0}}"
if [[ "${KEEP_PREP}" == "1" ]]; then
  ARGS+=(--keep-prepared)
fi

"${SCRIPT_DIR}/run_isaac_viz.sh" "${ARGS[@]}"
echo "Isaac viz smoke PASSED"
