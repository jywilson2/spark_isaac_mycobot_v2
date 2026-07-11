#!/usr/bin/env bash
# Resolve Isaac Sim python.sh for host-side scripts (build scene, live sim).
#
# Usage (from other scripts):
#   source "$(dirname "${BASH_SOURCE[0]}")/isaac_sim_env.sh"
#   require_isaac_python   # sets ISAACSIM_PYTHON_EXE and ISAACSIM_PATH
#
# Override detection with either:
#   export ISAACSIM_PATH=/path/to/release   # directory containing python.sh
#   export ISAACSIM_PYTHON_EXE=/path/to/python.sh

isaac_sim__candidate_paths() {
  local home="${HOME:-/root}"
  cat <<EOF
${ISAACSIM_PATH:+$ISAACSIM_PATH/python.sh}
${ISAACSIM_PYTHON_EXE:-}
${home}/IsaacSim/_build/linux-aarch64/release/python.sh
${home}/isaacsim/python.sh
${home}/isaac-sim/python.sh
/opt/nvidia/isaac-sim/python.sh
EOF
}

require_isaac_python() {
  local candidate resolved="" dir

  if [[ -n "${ISAACSIM_PYTHON_EXE:-}" && -x "${ISAACSIM_PYTHON_EXE}" ]]; then
    resolved="${ISAACSIM_PYTHON_EXE}"
  else
    while IFS= read -r candidate; do
      [[ -z "${candidate}" ]] && continue
      if [[ -x "${candidate}" ]]; then
        resolved="${candidate}"
        break
      fi
    done < <(isaac_sim__candidate_paths | awk '!seen[$0]++')
  fi

  if [[ -z "${resolved}" ]]; then
    echo "Isaac Sim python launcher (python.sh) not found." >&2
    echo >&2
    echo "Set ISAACSIM_PATH to the directory that contains python.sh, for example:" >&2
    echo "  export ISAACSIM_PATH=\"\$HOME/IsaacSim/_build/linux-aarch64/release\"  # DGX Spark source build" >&2
    echo "  export ISAACSIM_PATH=\"\$HOME/isaacsim\"                              # pre-built binary install" >&2
    echo >&2
    echo "Or set the executable directly:" >&2
    echo "  export ISAACSIM_PYTHON_EXE=\"/path/to/python.sh\"" >&2
    echo >&2
    echo "Search on the host:" >&2
    echo "  find \"\$HOME\" -name python.sh 2>/dev/null | grep -i isaac" >&2
    echo "  ls -l \"\$HOME/IsaacSim/_build/linux-aarch64/release/python.sh\"" >&2
    return 1
  fi

  ISAACSIM_PYTHON_EXE="${resolved}"
  dir="$(cd "$(dirname "${ISAACSIM_PYTHON_EXE}")" && pwd)"
  ISAACSIM_PATH="${dir}"
  export ISAACSIM_PYTHON_EXE ISAACSIM_PATH
}

print_isaac_sim_env() {
  require_isaac_python || return 1
  echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
  echo "ISAACSIM_PYTHON_EXE=${ISAACSIM_PYTHON_EXE}"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  print_isaac_sim_env
fi
