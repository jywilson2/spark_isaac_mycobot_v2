#!/usr/bin/env bash
# Compatibility wrapper: supervised residual is now Phase 3 (four-phase plan).
echo "NOTE: run_phase2_supervised.sh is deprecated; use scripts/run_phase3_supervised.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_phase3_supervised.sh" "$@"
