#!/usr/bin/env bash
# Probe which classical IK libraries are importable under Isaac Sim python.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
"${ISAACSIM_PATH}/python.sh" - <<'PY'
import importlib.util as u
for m in ("pinocchio", "scipy", "ikpy", "numpy", "torch", "curobo"):
    print(m, "OK" if u.find_spec(m) else "NO")
PY
