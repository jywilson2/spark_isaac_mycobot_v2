#!/usr/bin/env bash
# Unified verification entry point — CI (remote PR) vs Spark host (with Isaac Sim).
#
#   ./scripts/run_verification.sh ci       # remote GitHub PR / agent CI gate (no GUI)
#   ./scripts/run_verification.sh spark    # DGX Spark: CI suite, then required GUI smoke
#   ./scripts/run_verification.sh help
#
# Policy (spec.md Phase 1 Acceptance #7):
#   - Remote CI / PR: headless only — never require DISPLAY or --gui
#   - Spark host with Isaac Sim: after CI-equivalent passes, GUI smoke is required
#
# Agent rule: after Phase 1 / Isaac-path changes on this machine, run
#   ./scripts/run_verification.sh spark
# (from the container that delegates via spark_host_exec, or a host shell).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

MODE="${1:-}"
shift || true

usage() {
  cat <<'EOF'
Usage: ./scripts/run_verification.sh <ci|spark|help>

  ci      Remote GitHub PR / CI verification (headless only)
            1) pytest tests -q
            2) Optional headless Isaac smoke if SPARK_RUN_ISAAC_SMOKE=1
               or if --with-isaac is passed

  spark   DGX Spark host development (Isaac Sim available)
            1) Same as ci (pytest + headless Isaac smoke)
            2) Required GUI smoke (--gui, --auto-exit via smoke script)
               Delegates through spark_host_exec when run from the container

Options (after mode):
  --with-isaac     For ci: also run headless Isaac smoke (sets SPARK_RUN_ISAAC_SMOKE=1)
  --skip-pytest    Skip the NumPy unit suite (debug only)
  --skip-gui       For spark: stop after headless (not for declaring Spark verification done)

Examples:
  ./scripts/run_verification.sh ci
  ./scripts/run_verification.sh ci --with-isaac
  ./scripts/run_verification.sh spark
EOF
}

WITH_ISAAC=0
SKIP_PYTEST=0
SKIP_GUI=0
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-isaac) WITH_ISAAC=1; shift ;;
    --skip-pytest) SKIP_PYTEST=1; shift ;;
    --skip-gui) SKIP_GUI=1; shift ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      EXTRA+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${MODE}" || "${MODE}" == "help" || "${MODE}" == "-h" || "${MODE}" == "--help" ]]; then
  usage
  exit 0
fi

export SPARK_REPO_ROOT="${SPARK_REPO_ROOT:-${ROOT}}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export SPARK_HOST_USER="${SPARK_HOST_USER:-jywilson}"

run_pytest() {
  if [[ "${SKIP_PYTEST}" -eq 1 ]]; then
    echo "=== Skipping pytest (--skip-pytest) ==="
    return 0
  fi
  echo "=== CI: unit tests (pytest) ==="
  if [[ -f "${ROOT}/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${ROOT}/.venv/bin/activate"
  fi
  python3 -m pytest tests -q "${EXTRA[@]+"${EXTRA[@]}"}"
}

run_headless_isaac() {
  echo "=== CI: headless Isaac Sim smoke ==="
  if [[ -f /.dockerenv ]]; then
    bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/smoke_phase1_isaac.sh
  else
    bash "${ROOT}/scripts/host/smoke_phase1_isaac.sh"
  fi
}

run_gui_isaac() {
  echo "=== Spark: GUI Isaac Sim smoke (required after headless) ==="
  if [[ -f /.dockerenv ]]; then
    bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/smoke_phase1_isaac.sh --gui
  else
    bash "${ROOT}/scripts/host/smoke_phase1_isaac.sh" --gui
  fi
}

case "${MODE}" in
  ci)
    echo "############################################"
    echo "# Verification mode: CI / remote GitHub PR #"
    echo "# (headless only — no GUI)                 #"
    echo "############################################"
    run_pytest
    if [[ "${WITH_ISAAC}" -eq 1 || "${SPARK_RUN_ISAAC_SMOKE:-0}" == "1" ]]; then
      export SPARK_RUN_ISAAC_SMOKE=1
      run_headless_isaac
    else
      echo "NOTE: headless Isaac smoke not run (pass --with-isaac or SPARK_RUN_ISAAC_SMOKE=1)."
      echo "      Gated test remains skipped in plain pytest (expected for remote CI without Kit)."
    fi
    echo "=== CI verification PASSED ==="
    ;;
  spark)
    echo "############################################"
    echo "# Verification mode: DGX Spark + Isaac Sim #"
    echo "# (CI suite, then required GUI)            #"
    echo "############################################"
    run_pytest
    export SPARK_RUN_ISAAC_SMOKE=1
    run_headless_isaac
    if [[ "${SKIP_GUI}" -eq 1 ]]; then
      echo "WARNING: --skip-gui set; Spark verification is INCOMPLETE per Acceptance #7." >&2
      exit 2
    fi
    run_gui_isaac
    echo "=== Spark verification PASSED (pytest + headless + GUI) ==="
    ;;
  *)
    echo "ERROR: unknown mode '${MODE}'" >&2
    usage >&2
    exit 2
    ;;
esac
