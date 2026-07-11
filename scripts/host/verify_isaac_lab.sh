#!/usr/bin/env bash
# Run Isaac Lab unit tests and integration verification on the host.
#
# Usage:
#   ./scripts/host/verify_isaac_lab.sh
#   ./scripts/host/verify_isaac_lab.sh --smoke-env
#   ./scripts/host/verify_isaac_lab.sh --smoke-train
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
# shellcheck source=../../isaac_lab/versions.env
source "${REPO_ROOT}/isaac_lab/versions.env"

SMOKE_ENV=0
SMOKE_TRAIN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --smoke-env)
      SMOKE_ENV=1
      shift
      ;;
    --smoke-train)
      SMOKE_TRAIN=1
      shift
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

spark_host_apply_env
export ISAACLAB_PATH="${ISAACLAB_PATH:-${SPARK_ISAACLAB_PATH}}"
export SPARK_REPO_ROOT="${REPO_ROOT}"
export PYTEST_TMPDIR="${PYTEST_TMPDIR:-/tmp/spark_pytest_${USER:-$(id -un)}}"
mkdir -p "${PYTEST_TMPDIR}"

if [[ ! -x "${ISAACLAB_PATH}/isaaclab.sh" ]]; then
  echo "Isaac Lab not installed. Run: ${REPO_ROOT}/scripts/host/install_isaac_lab.sh" >&2
  exit 1
fi

echo "=== Container-safe unit tests (mdp + detect) ==="
python3 -m pytest \
  -o "basetemp=${PYTEST_TMPDIR}" \
  "${REPO_ROOT}/isaac_lab/test/test_mdp_contract.py" \
  "${REPO_ROOT}/isaac_lab/test/test_detect_isaac_lab.py" \
  "${REPO_ROOT}/isaac_lab/test/test_training_defaults.py" \
  -q

echo "=== Host Isaac Lab detect ==="
"${ISAACLAB_PATH}/isaaclab.sh" -p "${REPO_ROOT}/isaac_lab/detect_isaac_lab.py"

echo "=== Host Isaac Lab integration verify ==="
VERIFY_ARGS=(--headless)
if [[ "${SMOKE_ENV}" -eq 1 ]]; then
  VERIFY_ARGS+=(--smoke-env --steps 8)
fi
(
  cd "${ISAACLAB_PATH}"
  ./isaaclab.sh -p "${REPO_ROOT}/isaac_lab/verify_install.py" "${VERIFY_ARGS[@]}"
)

echo "=== Host pytest (integration gate) ==="
python3 -m pytest -o "basetemp=${PYTEST_TMPDIR}" \
  "${REPO_ROOT}/isaac_lab/test/test_isaac_lab_integration.py" -q

if [[ "${SMOKE_TRAIN}" -eq 1 ]]; then
  echo "=== Headless PPO integration train (DGX Spark: 8 arms, 30 min default) ==="
  (
    cd "${ISAACLAB_PATH}"
    ./isaaclab.sh -p "${REPO_ROOT}/isaac_lab/train_ppo.py" \
      --headless --viz none --enable_cameras \
      --num-arms 8
  )
fi

echo "Isaac Lab verification complete."
