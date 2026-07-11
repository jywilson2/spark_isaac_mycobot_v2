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
  if [[ -n "${display}" ]]; then
    echo "DISPLAY:   ${display}" >&2
  fi
  if [[ -n "${xauth}" ]]; then
    echo "XAUTHORITY: ${xauth}" >&2
  fi

  nsenter -t 1 -m -- env \
    SPARK_SKIP_HOST_DELEGATE=1 \
    SPARK_REPO_ROOT="${host_repo}" \
    HOME="${host_home}" \
    USER="${host_user}" \
    DISPLAY="${display}" \
    XAUTHORITY="${xauth}" \
    bash -lc "cd $(printf '%q' "${host_repo}") && bash $(printf '%q' "${host_script}") $(printf '%q ' "$@")"
}
