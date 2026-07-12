#!/usr/bin/env bash
# Host-side Isaac Sim environment for DGX Spark / native terminals.
#
# Source from host iteration scripts (NOT from the Isaac ROS container):
#   source /path/to/spark_isaac_mycobot_v2/scripts/host/env.isaac_host.sh
#   spark_host_check_prereqs
#
# Optional one-time host ~/.bashrc:
#   export ISAACSIM_PATH="$HOME/IsaacSim/_build/linux-aarch64/release"

if [[ -n "${SPARK_ISAAC_HOST_ENV_LOADED:-}" ]]; then
  return 0
fi
SPARK_ISAAC_HOST_ENV_LOADED=1

_SPARK_HOST_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPARK_REPO_ROOT="$(cd "${_SPARK_HOST_SCRIPT_DIR}/../.." && pwd)"

# shellcheck source=../isaac_sim_env.sh
source "${SPARK_REPO_ROOT}/scripts/isaac_sim_env.sh"

spark_host_require_native_shell() {
  if [[ -f /.dockerenv ]]; then
    echo "WARNING: /.dockerenv detected — run Isaac Sim scripts on the HOST terminal," >&2
    echo "not inside the Isaac ROS/Cursor container." >&2
    echo "  isaac-ros activate   # host" >&2
    echo "  ./scripts/host/iter_build_isaac_scene.sh   # host" >&2
    return 1
  fi
  return 0
}

spark_host_apply_env() {
  require_isaac_python || return 1

  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  # residual_adaptive_ik lives under src/; isaac_sim package lives at repo root
  export PYTHONPATH="${SPARK_REPO_ROOT}/src:${SPARK_REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

  if [[ -f /lib/aarch64-linux-gnu/libgomp.so.1 ]]; then
    case ":${LD_PRELOAD:-}:" in
      *:/lib/aarch64-linux-gnu/libgomp.so.1:*) ;;
      *)
        export LD_PRELOAD="${LD_PRELOAD:+$LD_PRELOAD:}/lib/aarch64-linux-gnu/libgomp.so.1"
        ;;
    esac
  fi

  export SPARK_HOST_LOG_DIR="${SPARK_HOST_LOG_DIR:-${SPARK_REPO_ROOT}/assets/logs/isaac_host}"
  mkdir -p "${SPARK_HOST_LOG_DIR}"
  return 0
}

spark_host_check_prereqs() {
  local urdf="${SPARK_REPO_ROOT}/third_party/mycobot_ros2/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf"
  local kin_urdf="${SPARK_REPO_ROOT}/assets/urdf/mycobot_280_m5_kinematics.urdf"
  local errors=0

  spark_host_apply_env || errors=$((errors + 1))

  if [[ ! -x "${ISAACSIM_PYTHON_EXE:-}" ]]; then
    echo "ERROR: Isaac Sim python.sh not found." >&2
    echo "  export ISAACSIM_PATH=\"\$HOME/isaacsim\"" >&2
    echo "  export ISAACSIM_PATH=\"\$HOME/IsaacSim/_build/linux-aarch64/release\"" >&2
    echo "  ./scripts/isaac_sim_env.sh" >&2
    errors=$((errors + 1))
  fi

  if [[ ! -f "${urdf}" ]]; then
    echo "WARNING: Vendor URDF+meshes missing: ${urdf}" >&2
    echo "  ./scripts/download_mycobot_ros2.sh" >&2
    if [[ ! -f "${kin_urdf}" ]]; then
      echo "ERROR: Neither vendor nor kinematics URDF found." >&2
      errors=$((errors + 1))
    else
      echo "  (kinematics-only asset present — NumPy IK OK; rendered import needs vendor meshes)" >&2
    fi
  fi

  if [[ "${errors}" -gt 0 ]]; then
    return 1
  fi

  echo "SPARK_REPO_ROOT=${SPARK_REPO_ROOT}"
  echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
  echo "ISAACSIM_PYTHON_EXE=${ISAACSIM_PYTHON_EXE}"
  echo "SPARK_HOST_LOG_DIR=${SPARK_HOST_LOG_DIR}"
  return 0
}

spark_host_new_log() {
  local label="${1:-isaac_host}"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  spark_host_apply_env >/dev/null 2>&1 || true
  local log_dir="${SPARK_HOST_LOG_DIR}"
  if ! mkdir -p "${log_dir}" 2>/dev/null; then
    log_dir="${TMPDIR:-/tmp}/spark_isaac_host_logs"
    mkdir -p "${log_dir}"
  fi
  local log_path="${log_dir}/${label}_${stamp}.log"
  ln -sfn "$(basename "${log_path}")" "${log_dir}/latest_${label}.log" 2>/dev/null || true
  ln -sfn "$(basename "${log_path}")" "${log_dir}/latest.log" 2>/dev/null || true
  printf '%s' "${log_path}"
}

spark_host_print_log_tail() {
  local log_path="${1:?log path required}"
  local lines="${2:-80}"
  echo "--- last ${lines} lines of ${log_path} ---"
  tail -n "${lines}" "${log_path}" || true
  echo "--- grep errors (if any) ---"
  grep -nE "Error|error|failed|Traceback|Scene build failed" "${log_path}" | tail -n 40 || true
}

spark_host_run_python() {
  spark_host_apply_env || return 1
  "${ISAACSIM_PYTHON_EXE}" "$@"
}
