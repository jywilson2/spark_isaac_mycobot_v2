#!/usr/bin/env bash
# Probe host Isaac python for torch / cuRobo.
#
# Exit 0 when cuRobo imports and CUDA is available; else exit 1.
# Used by ./scripts/run_verification.sh spark preflight.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
echo "USER=$(id -un) ISAACSIM_PYTHON_EXE=${ISAACSIM_PYTHON_EXE}"
"${ISAACSIM_PYTHON_EXE}" - <<'PY'
import sys
from pathlib import Path

print("python", sys.version)
ok = True
try:
    import torch

    print("torch", torch.__version__, "cuda", torch.cuda.is_available())
    if not torch.cuda.is_available():
        ok = False
except Exception as e:
    print("torch FAIL", e)
    ok = False
try:
    import curobo

    print("curobo", getattr(curobo, "__version__", "ok"), curobo.__file__)
except Exception as e:
    print("curobo FAIL", e)
    ok = False

home = Path.home()
for p in [
    home / "curobo",
    home / "pkgs" / "curobo",
    home / "workspaces" / "curobo",
    Path("/home/jywilson/curobo"),
]:
    print("path_exists", p, p.exists())

sys.exit(0 if ok else 1)
PY
