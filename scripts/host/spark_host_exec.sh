#!/usr/bin/env bash
# Delegate Isaac Sim / Isaac Lab commands to the DGX Spark **host** when the
# caller is inside the Isaac ROS container (Cursor agent or `docker exec`).
#
# Isaac Sim is not installed in the container. Historically, agents ran host
# scripts via `nsenter -t 1 -m` into PID 1's mount namespace so training and
# scene builds execute with the host's `~/isaacsim` and `~/IsaacLab` installs.
#
# Tutorial: spec.md § Host vs container execution
# See also: README.md § Development Workflow (isaac-ros activate)

spark_in_isaac_ros_container() {
  [[ -f /.dockerenv ]]
}

spark_resolve_host_repo_root() {
  if [[ -n "${SPARK_HOST_REPO_ROOT:-}" ]]; then
    printf '%s\n' "${SPARK_HOST_REPO_ROOT}"
    return 0
  fi
  if ! spark_in_isaac_ros_container; then
    printf '%s\n' "${SPARK_REPO_ROOT:?SPARK_REPO_ROOT not set}"
    return 0
  fi
  local container_root="${SPARK_REPO_ROOT:-}"
  if command -v nsenter >/dev/null 2>&1; then
    local host_user="${SPARK_HOST_USER:-admin}"
    local candidates=(
      "/home/${host_user}/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2"
      "/home/jywilson/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2"
      "${container_root}"
    )
    local path
    for path in "${candidates[@]}"; do
      if nsenter -t 1 -m -- test -f "${path}/spec.md" 2>/dev/null; then
        printf '%s\n' "${path}"
        return 0
      fi
    done
  fi
  printf '%s\n' "${container_root}"
}

# Resolve DISPLAY / XAUTHORITY for GUI modes (demo, play, train with viz).
# Prints one KEY=VALUE per line for easy testing.
spark_resolve_host_gui_env() {
  local host_home="${1:?host_home required}"
  local display="${DISPLAY:-}"
  local xauth="${XAUTHORITY:-}"

  if [[ -z "${display}" ]] && command -v nsenter >/dev/null 2>&1; then
    display="$(nsenter -t 1 -m -- bash -lc 'printf %s "${DISPLAY:-}"' 2>/dev/null || true)"
  fi
  if [[ -z "${display}" ]]; then
    local sock
    for sock in /tmp/.X11-unix/X*; do
      [[ -S "${sock}" ]] || continue
      display=":${sock##*/X}"
      break
    done
  fi

  if [[ -z "${xauth}" ]]; then
    if spark_in_isaac_ros_container && command -v nsenter >/dev/null 2>&1; then
      if nsenter -t 1 -m -- test -f "${host_home}/.Xauthority" 2>/dev/null; then
        xauth="${host_home}/.Xauthority"
      fi
    elif [[ -f "${host_home}/.Xauthority" ]]; then
      xauth="${host_home}/.Xauthority"
    fi
  fi

  printf 'DISPLAY=%s\n' "${display}"
  printf 'XAUTHORITY=%s\n' "${xauth}"
}

spark_require_gui_display() {
  local host_home="${1:-${HOME}}"
  local display="" xauth="" line key value
  while IFS= read -r line; do
    key="${line%%=*}"
    value="${line#*=}"
    case "${key}" in
      DISPLAY) display="${value}" ;;
      XAUTHORITY) xauth="${value}" ;;
    esac
  done < <(spark_resolve_host_gui_env "${host_home}")

  if [[ -z "${display}" ]]; then
    echo "ERROR: DISPLAY is not set — Isaac Sim GUI (demo/play) requires an X11 session." >&2
    echo "  Run from a host graphical terminal, or ensure xhost + container X11 forwarding." >&2
    return 1
  fi
  if [[ -n "${xauth}" && ! -f "${xauth}" ]]; then
    echo "ERROR: XAUTHORITY file missing: ${xauth}" >&2
    return 1
  fi
  export DISPLAY="${display}"
  if [[ -n "${xauth}" ]]; then
    export XAUTHORITY="${xauth}"
  fi
  return 0
}

spark_delegate_to_host() {
  local host_script="$1"
  shift

  if ! spark_in_isaac_ros_container; then
    echo "spark_delegate_to_host: not in container; run natively instead." >&2
    return 1
  fi
  if ! command -v nsenter >/dev/null 2>&1; then
    echo "ERROR: nsenter not available — cannot reach Isaac Sim on the host." >&2
    echo "Run from a native host terminal after: isaac-ros activate" >&2
    echo "  ${host_script} $*" >&2
    return 1
  fi

  local host_repo host_user host_home
  host_repo="$(spark_resolve_host_repo_root)"
  if [[ "${host_repo}" =~ ^/home/([^/]+)/ ]]; then
    host_user="${BASH_REMATCH[1]}"
  else
    host_user="${SPARK_HOST_USER:-admin}"
  fi
  host_home="/home/${host_user}"

  local display="" xauth="" line key value
  while IFS= read -r line; do
    key="${line%%=*}"
    value="${line#*=}"
    case "${key}" in
      DISPLAY) display="${value}" ;;
      XAUTHORITY) xauth="${value}" ;;
    esac
  done < <(spark_resolve_host_gui_env "${host_home}")

  # host_script must be relative to host_repo (e.g. ./scripts/host/...)
  case "${host_script}" in
    /*)
      echo "ERROR: host_script must be repo-relative, not an absolute container path." >&2
      echo "  Got: ${host_script}" >&2
      return 1
      ;;
  esac

  echo "=== Delegating to Isaac Sim host (nsenter) ===" >&2
  echo "Host repo: ${host_repo}" >&2
  echo "Command:   ${host_script} $*" >&2
  echo "Host user: ${host_user} (non-root via runuser when available)" >&2
  if [[ -n "${display}" ]]; then
    echo "DISPLAY:   ${display}" >&2
  fi
  if [[ -n "${xauth}" ]]; then
    echo "XAUTHORITY: ${xauth}" >&2
  fi

  local arg_str=""
  if [[ $# -gt 0 ]]; then
    arg_str="$(printf ' %q' "$@")"
  fi

  # Build the inner command (repo-relative script). Isaac Sim GUI needs the
  # desktop session owner's UID so X11 MIT-MAGIC-COOKIE auth succeeds — v1 only
  # set HOME/USER while remaining root, which often yields
  # "Authorization required, but no authorization protocol specified".
  local inner="cd $(printf '%q' "${host_repo}") && bash $(printf '%q' "${host_script}")${arg_str}"
  local isaac_path="${ISAACSIM_PATH:-${host_home}/isaacsim}"
  local isaac_py="${ISAACSIM_PYTHON_EXE:-}"

  local -a ns_env=(
    SPARK_SKIP_HOST_DELEGATE=1
    SPARK_REPO_ROOT="${host_repo}"
    SPARK_ALLOW_CONTAINER_ISAAC=0
    HOME="${host_home}"
    USER="${host_user}"
    LOGNAME="${host_user}"
    DISPLAY="${display}"
    XAUTHORITY="${xauth}"
    ISAACSIM_PATH="${isaac_path}"
    ISAACSIM_PYTHON_EXE="${isaac_py}"
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"
  )
  # Forward optional Phase 1 smoke knobs (headless metrics-only on Spark, etc.).
  local _fwd
  for _fwd in ISAAC_VIZ_SMOKE_VISUALIZE ISAAC_VIZ_SMOKE_N_POSES ISAAC_VIZ_SMOKE_HOLD_S \
    ISAAC_VIZ_SMOKE_KEEP_GUI_OPEN ISAAC_VIZ_SMOKE_KEEP_PREPARED ISAAC_VIZ_SMOKE_RESET_TO_HOME \
    ISAAC_VIZ_MIN_PLAN_OK_RATE \
    PHASE1_SMOKE_VISUALIZE PHASE1_SMOKE_N_POSES PHASE1_SMOKE_HOLD_S \
    PHASE1_SMOKE_KEEP_GUI_OPEN PHASE1_SMOKE_KEEP_PREPARED PHASE1_SMOKE_RESET_TO_HOME; do
    if [[ -n "${!_fwd:-}" ]]; then
      ns_env+=("${_fwd}=${!_fwd}")
    fi
  done

  # Drop to the host desktop user when possible (X11 + writable home).
  # Override with SPARK_HOST_RUN_AS_USER=0 to force root (debug only).
  local run_as="${SPARK_HOST_RUN_AS_USER:-1}"

  # Ensure the host user can write assets/logs created by earlier root smokes.
  if [[ "${run_as}" == "1" ]]; then
    nsenter -t 1 -m -- bash -lc \
      "chown -R $(printf '%q' "${host_user}:${host_user}") \
        $(printf '%q' "${host_repo}/assets") \
        $(printf '%q' "${host_repo}/docs") 2>/dev/null || true"
  fi

  if [[ "${run_as}" == "1" ]] && nsenter -t 1 -m -- id -u "${host_user}" >/dev/null 2>&1; then
    if nsenter -t 1 -m -- test -x /usr/sbin/runuser; then
      nsenter -t 1 -m -- /usr/sbin/runuser -u "${host_user}" -- env "${ns_env[@]}" \
        bash -lc "${inner}"
      return $?
    fi
    if nsenter -t 1 -m -- test -x /usr/bin/setpriv; then
      local host_uid host_gid
      host_uid="$(nsenter -t 1 -m -- id -u "${host_user}")"
      host_gid="$(nsenter -t 1 -m -- id -g "${host_user}")"
      nsenter -t 1 -m -- /usr/bin/setpriv \
        --reuid="${host_uid}" --regid="${host_gid}" --init-groups -- \
        env "${ns_env[@]}" bash -lc "${inner}"
      return $?
    fi
    echo "WARNING: runuser/setpriv unavailable; continuing as root (GUI X11 may fail)." >&2
  fi

  nsenter -t 1 -m -- env "${ns_env[@]}" bash -lc "${inner}"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  # CLI: ./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh [args...]
  if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <repo-relative-script> [args...]" >&2
    exit 2
  fi
  _target="$1"
  shift
  export SPARK_REPO_ROOT="${SPARK_REPO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
  export SPARK_HOST_USER="${SPARK_HOST_USER:-jywilson}"
  export ISAACSIM_PATH="${ISAACSIM_PATH:-/home/${SPARK_HOST_USER}/isaacsim}"
  if spark_in_isaac_ros_container && [[ "${SPARK_SKIP_HOST_DELEGATE:-0}" != "1" ]]; then
    spark_delegate_to_host "${_target}" "$@"
  else
    cd "${SPARK_REPO_ROOT}"
    bash "${_target}" "$@"
  fi
fi
