#!/usr/bin/env bash
# Install NVIDIA cuRobo (Apache-2.0) into the Isaac Sim python environment.
#
# Host-only. Uses Isaac Sim's python.sh so Kit and cuRobo share the same torch/CUDA.
# See https://curobo.org/get_started/1_install_instructions.html and spec.md Phase 2.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1

if [[ -f /.dockerenv && "${SPARK_ALLOW_CONTAINER_ISAAC:-0}" != "1" ]]; then
  echo "Install cuRobo on the host (or via spark_host_exec)." >&2
  exit 2
fi

CUROBO_DIR="${CUROBO_DIR:-${HOME}/curobo}"
CUROBO_REF="${CUROBO_REF:-v0.7.8}"
PY="${ISAACSIM_PYTHON_EXE}"

echo "=== Install cuRobo ${CUROBO_REF} into ${PY} ==="
echo "CUROBO_DIR=${CUROBO_DIR}"

"${PY}" -m pip install -q tomli wheel ninja

if [[ ! -d "${CUROBO_DIR}/.git" ]]; then
  rm -rf "${CUROBO_DIR}"
  git clone --depth 1 --branch "${CUROBO_REF}" https://github.com/NVlabs/curobo.git "${CUROBO_DIR}"
else
  git -C "${CUROBO_DIR}" fetch --depth 1 origin "refs/tags/${CUROBO_REF}:refs/tags/${CUROBO_REF}" || true
  git -C "${CUROBO_DIR}" checkout "${CUROBO_REF}"
fi

# Prefer isaacsim extra when available; fall back to plain editable install.
set +e
"${PY}" -m pip install -e "${CUROBO_DIR}[isaacsim]" --no-build-isolation
rc=$?
set -e
if [[ "${rc}" -ne 0 ]]; then
  echo "NOTE: pip install [isaacsim] failed; retrying without extra..."
  "${PY}" -m pip install -e "${CUROBO_DIR}" --no-build-isolation
fi

"${PY}" - <<'PY'
import curobo
print("cuRobo OK:", getattr(curobo, "__version__", "unknown"), curobo.__file__)
PY

echo "cuRobo install complete."
