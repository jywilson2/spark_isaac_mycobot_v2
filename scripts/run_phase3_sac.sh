#!/usr/bin/env bash
# Compatibility wrapper: SAC residual is now Phase 4 (four-phase plan).
echo "NOTE: run_phase3_sac.sh is deprecated; use scripts/run_phase4_sac.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_phase4_sac.sh" "$@"
