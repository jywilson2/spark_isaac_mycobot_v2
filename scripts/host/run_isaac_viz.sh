#!/usr/bin/env bash
# Host Isaac Sim IK viz: Phase 1 DLS metrics + Phase 2 collision-aware planning.
#
#   ./scripts/download_mycobot_ros2.sh
#   ./scripts/host/run_isaac_viz.sh
#
# Quick visual smoke (240 metric poses ≈ one per workspace bin, animate 48):
#   ./scripts/host/run_isaac_viz.sh --skip-tests -- \
#       --num-poses 240 --visualize 48 --hold-s 0.4
#
# Full acceptance (≥1000 poses) with rendering of a subset:
#   ./scripts/host/run_isaac_viz.sh -- \
#       --num-poses 1000 --visualize 48 --hold-s 0.75
#
# Legacy alias: scripts/host/run_isaac_viz.sh → this script.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"

SKIP_TESTS=0
VIZ_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-tests)
      SKIP_TESTS=1
      shift
      ;;
    --skip-metrics)
      echo "NOTE: --skip-metrics is obsolete; metrics are computed inside the Isaac run." >&2
      echo "      Use --visualize 0 or --metrics-only after -- if you only want numbers." >&2
      shift
      ;;
    --)
      shift
      VIZ_ARGS+=("$@")
      break
      ;;
    *)
      VIZ_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ -f /.dockerenv && "${SPARK_ALLOW_CONTAINER_ISAAC:-0}" != "1" ]]; then
  echo "Isaac Sim IK viz must run on the **host**." >&2
  echo "  ${SPARK_REPO_ROOT}/scripts/host/run_isaac_viz.sh" >&2
  echo "(Use a native host shell with Isaac Sim installed — not isaac-ros activate / the ROS container.)" >&2
  exit 2
fi

spark_host_require_native_shell || true
spark_host_apply_env || exit 1

if [[ ! -f "${SPARK_REPO_ROOT}/third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf" ]]; then
  echo "Fetching mycobot_ros2 (URDF + meshes)..."
  "${SPARK_REPO_ROOT}/scripts/download_mycobot_ros2.sh"
fi

export SPARK_REPO_ROOT
export PYTHONPATH="${SPARK_REPO_ROOT}/src:${SPARK_REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

HOST_PY="${SPARK_HOST_PYTHON:-python3}"
# Defaults (ISAAC_VIZ_*; legacy PHASE1_* still accepted)
N_POSES="${ISAAC_VIZ_N_POSES:-${PHASE1_N_POSES:-1000}}"
N_VIZ="${ISAAC_VIZ_VISUALIZE:-${PHASE1_VISUALIZE:-48}}"
SEED="${ISAAC_VIZ_SEED:-${PHASE1_SEED:-0}}"

if [[ "${SKIP_TESTS}" -eq 0 ]]; then
  echo "=== Unit tests before Isaac viz (host python) ==="
  if command -v pytest >/dev/null 2>&1; then
    (cd "${SPARK_REPO_ROOT}" && "${HOST_PY}" -m pytest \
      tests/test_fk.py tests/test_ik_validation.py tests/test_residual_bounds.py \
      tests/test_urdf_utils.py -q)
  else
    echo "pytest not found on PATH — skipping unit tests (install or --skip-tests)" >&2
  fi
fi

# Inject defaults only when the user did not already pass the flag
has_flag() {
  local flag="$1"
  local a
  for a in "${VIZ_ARGS[@]+"${VIZ_ARGS[@]}"}"; do
    [[ -n "${a}" && "${a}" == "${flag}" ]] && return 0
  done
  return 1
}

# Drop accidental empty argv entries (can appear via remote/shell quoting).
_filtered=()
for _a in "${VIZ_ARGS[@]+"${VIZ_ARGS[@]}"}"; do
  [[ -n "${_a}" ]] && _filtered+=("${_a}")
done
VIZ_ARGS=("${_filtered[@]+"${_filtered[@]}"}")
unset _filtered _a

if ! has_flag --num-poses; then
  VIZ_ARGS+=(--num-poses "${N_POSES}")
fi
if ! has_flag --visualize; then
  VIZ_ARGS+=(--visualize "${N_VIZ}")
fi
if ! has_flag --seed; then
  VIZ_ARGS+=(--seed "${SEED}")
fi

LOG_PATH="$(spark_host_new_log isaac_viz_metrics)"
echo "=== Isaac Sim IK viz (Phase 1 metrics + Phase 2 planning) ==="
echo "ISAACSIM_PYTHON_EXE=${ISAACSIM_PYTHON_EXE}"
echo "Args: ${VIZ_ARGS[*]}"
echo "Writing log: ${LOG_PATH}"

set +e
spark_host_run_python \
  "${SPARK_REPO_ROOT}/isaac_sim/run_ik_viz.py" \
  --repo-root "${SPARK_REPO_ROOT}" \
  ${VIZ_ARGS[@]+"${VIZ_ARGS[@]}"} 2>&1 | tee "${LOG_PATH}"
exit_code="${PIPESTATUS[0]}"
set -e

if [[ "${exit_code}" -ne 0 ]]; then
  echo "Isaac viz FAILED (exit ${exit_code})" >&2
  spark_host_print_log_tail "${LOG_PATH}" 80
  exit "${exit_code}"
fi

echo "Isaac viz host run complete (metrics written during Isaac session)."
echo "Metrics: ${SPARK_REPO_ROOT}/docs/phase1_baseline.md"
echo "JSON:    ${SPARK_REPO_ROOT}/assets/logs/phase1_baseline_metrics.json"
echo "Log:     ${LOG_PATH}"
