#!/usr/bin/env bash
# Deprecated alias — use ./scripts/host/run_isaac_viz.sh
echo "NOTE: run_phase1_isaac.sh is deprecated; forwarding to run_isaac_viz.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_isaac_viz.sh" "$@"
