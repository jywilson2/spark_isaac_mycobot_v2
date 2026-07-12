#!/usr/bin/env bash
# Phase 1 baseline: unit tests + ≥1000-pose DLS IK metrics.
# Optional Isaac Sim visualization on the host:
#   PHASE1_WITH_ISAAC=1 ./scripts/run_phase1_baseline.sh
#   ./scripts/run_phase1_baseline.sh --with-isaac
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

WITH_ISAAC=0
FORWARD=()
for arg in "$@"; do
  case "${arg}" in
    --with-isaac)
      WITH_ISAAC=1
      ;;
    *)
      FORWARD+=("${arg}")
      ;;
  esac
done
if [[ "${PHASE1_WITH_ISAAC:-0}" == "1" ]]; then
  WITH_ISAAC=1
fi

if [[ "${WITH_ISAAC}" -eq 1 ]]; then
  exec "${ROOT}/scripts/host/run_phase1_isaac.sh" "${FORWARD[@]}"
fi

if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export SPARK_REPO_ROOT="${ROOT}"
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

N_POSES="${PHASE1_N_POSES:-1000}"
SEED="${PHASE1_SEED:-0}"

pytest tests/test_fk.py tests/test_ik_validation.py tests/test_residual_bounds.py tests/test_urdf_utils.py -q
python -m residual_adaptive_ik.kinematics.baseline_eval \
  --n-poses "${N_POSES}" \
  --seed "${SEED}" \
  --json-out "${ROOT}/assets/logs/phase1_baseline_metrics.json" \
  --md-out "${ROOT}/docs/phase1_baseline.md"

echo
echo "Phase 1 baseline complete. See docs/phase1_baseline.md"
echo "For Isaac Sim rendering on the host:"
echo "  ./scripts/host/run_phase1_isaac.sh"
echo "  ./scripts/host/launch_isaac_sim.sh"
