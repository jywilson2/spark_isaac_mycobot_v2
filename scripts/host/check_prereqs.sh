#!/usr/bin/env bash
# Quick host prerequisite check for Isaac Sim scene build iteration.
# Run on the DGX Spark HOST (native terminal):
#   ./scripts/host/check_prereqs.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"

spark_host_require_native_shell || true
spark_host_check_prereqs

echo
echo "Optional next steps:"
echo "  ./scripts/host/iter_urdf_import.sh      # fast URDF-only probe"
echo "  ./scripts/host/iter_build_isaac_scene.sh   # full scene build"
