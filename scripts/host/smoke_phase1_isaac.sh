#!/usr/bin/env bash
# Deprecated alias — use ./scripts/host/smoke_isaac_viz.sh
echo "NOTE: smoke_phase1_isaac.sh is deprecated; forwarding to smoke_isaac_viz.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/smoke_isaac_viz.sh" "$@"
