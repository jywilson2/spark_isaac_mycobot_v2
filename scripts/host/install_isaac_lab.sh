#!/usr/bin/env bash
# Install and verify Isaac Lab on the Isaac Sim host (required for Phase 2 PPO).
#
# Prerequisites:
#   - Isaac Sim pre-built at ~/isaacsim (or ISAACSIM_PATH)
#   - git, cmake, build-essential
#
# Usage:
#   ./scripts/host/install_isaac_lab.sh
#   ./scripts/host/install_isaac_lab.sh --verify-only
#   ISAACLAB_PATH=$HOME/IsaacLab ./scripts/host/install_isaac_lab.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
# shellcheck source=../../isaac_lab/versions.env
source "${REPO_ROOT}/isaac_lab/versions.env"

VERIFY_ONLY=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --verify-only)
      VERIFY_ONLY=1
      shift
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Usage: $0 [--verify-only]" >&2
      exit 1
      ;;
  esac
done

spark_host_require_native_shell || true
spark_host_check_prereqs

export ISAACLAB_PATH="${ISAACLAB_PATH:-${SPARK_ISAACLAB_PATH}}"
export SPARK_REPO_ROOT="${REPO_ROOT}"

LOG_PATH="${TMPDIR:-/tmp}/spark_install_isaac_lab_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to ${LOG_PATH}"
exec > >(tee -a "${LOG_PATH}") 2>&1

echo "=== Isaac Lab install ==="
echo "Log: ${LOG_PATH}"
echo "ISAACSIM_PATH=${ISAACSIM_PATH}"
echo "SPARK_ISAACLAB_BRANCH=${SPARK_ISAACLAB_BRANCH}"
echo "SPARK_ISAACLAB_RL_FRAMEWORK=${SPARK_ISAACLAB_RL_FRAMEWORK}"

if [[ "${VERIFY_ONLY}" -eq 0 ]]; then
  if [[ ! -d "${ISAACLAB_PATH}/.git" ]]; then
    echo "Cloning Isaac Lab to ${ISAACLAB_PATH}..."
    git clone "${SPARK_ISAACLAB_REPO}" "${ISAACLAB_PATH}" --branch "${SPARK_ISAACLAB_BRANCH}" --depth 1
  else
    echo "Isaac Lab already cloned at ${ISAACLAB_PATH}"
    git -C "${ISAACLAB_PATH}" fetch origin "${SPARK_ISAACLAB_BRANCH}" --depth 1 || true
    git -C "${ISAACLAB_PATH}" checkout "${SPARK_ISAACLAB_BRANCH}" || true
    if [[ "$(git -C "${ISAACLAB_PATH}" rev-parse --abbrev-ref HEAD)" != "${SPARK_ISAACLAB_BRANCH}" ]]; then
      echo "Checking out Isaac Lab branch ${SPARK_ISAACLAB_BRANCH}..."
      git -C "${ISAACLAB_PATH}" checkout -B "${SPARK_ISAACLAB_BRANCH}" \
        "origin/${SPARK_ISAACLAB_BRANCH}" 2>/dev/null \
        || git -C "${ISAACLAB_PATH}" checkout "${SPARK_ISAACLAB_BRANCH}"
    fi
  fi

  if [[ ! -e "${ISAACLAB_PATH}/_isaac_sim" ]]; then
    echo "Linking ${ISAACLAB_PATH}/_isaac_sim -> ${ISAACSIM_PATH}"
    ln -sfn "${ISAACSIM_PATH}" "${ISAACLAB_PATH}/_isaac_sim"
  fi

  local conda_stub="${ISAACLAB_PATH}/_isaac_sim/setup_conda_env.sh"
  if [[ ! -f "${conda_stub}" ]]; then
    echo "Creating Isaac Sim setup_conda_env.sh stub for pre-built binary installs"
    cat > "${conda_stub}" <<'EOF'
#!/usr/bin/env bash
# Stub for pre-built Isaac Sim installs without bundled conda env.
return 0 2>/dev/null || exit 0
EOF
    chmod +x "${conda_stub}"
  fi

  if ! command -v cmake >/dev/null 2>&1; then
    echo "Installing cmake/build-essential (required by rsl_rl/robomimic deps)..."
    sudo apt-get update -qq
    sudo apt-get install -y cmake build-essential
  fi

  echo "Installing Isaac Lab extensions + ${SPARK_ISAACLAB_RL_FRAMEWORK}..."
  (
    cd "${ISAACLAB_PATH}"
    export TERM="${TERM:-xterm-256color}"
    ./isaaclab.sh --install "${SPARK_ISAACLAB_RL_FRAMEWORK}"
  )
fi

echo "=== Verify Isaac Lab imports ==="
(
  cd "${ISAACLAB_PATH}"
  ./isaaclab.sh -p "${REPO_ROOT}/isaac_lab/detect_isaac_lab.py"
)

echo "=== Verify repo Isaac Lab integration (imports) ==="
(
  cd "${ISAACLAB_PATH}"
  export TERM="${TERM:-xterm-256color}"
  ./isaaclab.sh -p "${REPO_ROOT}/isaac_lab/verify_install.py" --headless
) || exit 1

echo "=== Verify headless env smoke ==="
(
  cd "${ISAACLAB_PATH}"
  export TERM="${TERM:-xterm-256color}"
  ./isaaclab.sh -p "${REPO_ROOT}/isaac_lab/verify_install.py" --headless --smoke-env --steps 4
) || exit 1

echo "=== Isaac Lab install verified ==="
echo "Next: ./scripts/host/run_isaac_lab_training.sh train --max-iterations 10"
