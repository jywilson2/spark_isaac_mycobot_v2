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
            0) Preflight: refuse if another Kit viz is running; require cuRobo
            1) pytest tests -q
            2) Headless Isaac metrics smoke (no arm animation; visualize=0)
            3) Phase 2 cuRobo MotionGen smoke
            4) Required GUI smoke (--gui, --auto-exit via smoke script)
               Delegates through spark_host_exec when run from the container

            One-time host setup: ./scripts/host/spark_host_exec.sh ./scripts/host/install_curobo.sh

            Tip: do not pipe this script through head/tail — that can SIGPIPE the
            parent while leaving an orphan Kit process on the GPU.

Options (after mode):
  --with-isaac     For ci: also run headless Isaac smoke (sets SPARK_RUN_ISAAC_SMOKE=1)
  --skip-pytest    Skip the NumPy unit suite (debug only)
  --skip-gui       For spark: stop after cuRobo smoke (exit 2; incomplete Spark gate)

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
  # basetemp is UID-scoped via tests/conftest.py (avoids root vs host /tmp clash).
  python3 -m pytest tests -q "${EXTRA[@]+"${EXTRA[@]}"}"
}

run_headless_isaac() {
  echo "=== CI: headless Isaac Sim smoke ==="
  # Headless runs the same Phase 2 planning / contact / rate-gate workload as
  # GUI (same n_poses / visualize / reset policy). Only the Kit window is
  # omitted; articulation playback is time-warped (default 4×) because no human
  # is watching. Scope env to this invocation so GUI stays real-time.
  local -a cmd
  if [[ -f /.dockerenv ]]; then
    cmd=(bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/smoke_isaac_viz.sh)
  else
    cmd=(bash "${ROOT}/scripts/host/smoke_isaac_viz.sh")
  fi
  if [[ "${MODE}" == "spark" ]]; then
    # Default matches GUI episode count (48). Override with
    # ISAAC_VIZ_SMOKE_HEADLESS_VISUALIZE if a shorter headless pass is needed.
    local viz="${ISAAC_VIZ_SMOKE_HEADLESS_VISUALIZE:-${PHASE1_SMOKE_HEADLESS_VISUALIZE:-${ISAAC_VIZ_SMOKE_GUI_VISUALIZE:-${ISAAC_VIZ_SMOKE_VISUALIZE:-48}}}}"
    local nposes="${ISAAC_VIZ_SMOKE_N_POSES:-${PHASE1_SMOKE_N_POSES:-240}}"
    local warp="${ISAAC_VIZ_SMOKE_TIME_WARP:-${PHASE1_SMOKE_TIME_WARP:-4}}"
    echo "NOTE: Spark headless = full planning (visualize=${viz}, n_poses=${nposes}, time_warp=${warp}×; no window)."
    ISAAC_VIZ_SMOKE_VISUALIZE="${viz}" PHASE1_SMOKE_VISUALIZE="${viz}" \
      ISAAC_VIZ_SMOKE_N_POSES="${nposes}" PHASE1_SMOKE_N_POSES="${nposes}" \
      ISAAC_VIZ_SMOKE_TIME_WARP="${warp}" PHASE1_SMOKE_TIME_WARP="${warp}" \
      ISAAC_VIZ_SMOKE_RESET_TO_HOME=0 PHASE1_SMOKE_RESET_TO_HOME=0 \
      "${cmd[@]}"
  else
    # CI default: same visualize count as smoke script (48) + headless time-warp.
    local warp="${ISAAC_VIZ_SMOKE_TIME_WARP:-${PHASE1_SMOKE_TIME_WARP:-4}}"
    ISAAC_VIZ_SMOKE_TIME_WARP="${warp}" PHASE1_SMOKE_TIME_WARP="${warp}" "${cmd[@]}"
  fi
}

run_gui_isaac() {
  echo "=== Spark: GUI Isaac Sim smoke (required after headless) ==="
  # Explicit GUI visualize count (default 48); never inherit headless 0.
  # Required policy (spec.md): home once at session start / first episode only.
  # Force sequential multi-target — never pass --reset-to-home here.
  local viz="${ISAAC_VIZ_SMOKE_GUI_VISUALIZE:-${PHASE1_SMOKE_GUI_VISUALIZE:-${ISAAC_VIZ_SMOKE_VISUALIZE:-${PHASE1_SMOKE_VISUALIZE:-48}}}}"
  echo "NOTE: Spark GUI uses ISAAC_VIZ_SMOKE_VISUALIZE=${viz} (home once; --no-reset-to-home)."
  local -a gui_env=(
    "ISAAC_VIZ_SMOKE_VISUALIZE=${viz}"
    "PHASE1_SMOKE_VISUALIZE=${viz}"
    "ISAAC_VIZ_SMOKE_RESET_TO_HOME=0"
    "PHASE1_SMOKE_RESET_TO_HOME=0"
  )
  if [[ -f /.dockerenv ]]; then
    env "${gui_env[@]}" \
      bash "${ROOT}/scripts/host/spark_host_exec.sh" \
      ./scripts/host/smoke_isaac_viz.sh --gui
  else
    env "${gui_env[@]}" \
      bash "${ROOT}/scripts/host/smoke_isaac_viz.sh" --gui
  fi
}

run_curobo_smoke() {
  echo "=== Spark: Phase 2 cuRobo MotionGen smoke ==="
  if [[ -f /.dockerenv ]]; then
    bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/smoke_phase2_curobo.sh
    bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/verify_target_obstacle.sh
    bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/diagnose_plan_recovery.sh --num-trials 12
  else
    bash "${ROOT}/scripts/host/smoke_phase2_curobo.sh"
    bash "${ROOT}/scripts/host/verify_target_obstacle.sh"
    bash "${ROOT}/scripts/host/diagnose_plan_recovery.sh" --num-trials 12
  fi
}

spark_preflight() {
  echo "=== Spark preflight ==="
  # A previous `... | head` / Ctrl+C can leave Kit holding the GPU; refuse early.
  if pgrep -f 'run_ik_viz.py|run_phase1_ik_viz.py' >/dev/null 2>&1; then
    echo "ERROR: another Isaac viz (run_ik_viz.py) is already running." >&2
    echo "  Kill the orphan Kit (or wait for it), then re-run." >&2
    echo "  Example: pgrep -af 'run_ik_viz.py|kit'" >&2
    exit 1
  fi
  # Fail fast before a multi-minute Isaac launch when cuRobo is missing.
  if [[ -f /.dockerenv ]]; then
    if ! bash "${ROOT}/scripts/host/spark_host_exec.sh" ./scripts/host/probe_curobo.sh; then
      echo "ERROR: cuRobo not available on the host Isaac python." >&2
      echo "  One-time: ./scripts/host/spark_host_exec.sh ./scripts/host/install_curobo.sh" >&2
      exit 1
    fi
  else
    if ! bash "${ROOT}/scripts/host/probe_curobo.sh"; then
      echo "ERROR: cuRobo not available in Isaac python." >&2
      echo "  One-time: ./scripts/host/install_curobo.sh" >&2
      exit 1
    fi
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
    echo "# (CI suite, then cuRobo, then required GUI) #"
    echo "############################################"
    spark_preflight
    # Avoid double GUI: pytest auto-runs GUI on Spark; verification owns the
    # GUI stage below so headless → cuRobo → GUI stay ordered.
    export SPARK_RUN_ISAAC_GUI_SMOKE=0
    run_pytest
    export SPARK_RUN_ISAAC_SMOKE=1
    run_headless_isaac
    run_curobo_smoke
    if [[ "${SKIP_GUI}" -eq 1 ]]; then
      echo "WARNING: --skip-gui set; Spark verification is INCOMPLETE per Acceptance #7." >&2
      exit 2
    fi
    run_gui_isaac
    echo "=== Spark verification PASSED (pytest + headless + cuRobo + GUI) ==="
    ;;
  *)
    echo "ERROR: unknown mode '${MODE}'" >&2
    usage >&2
    exit 2
    ;;
esac
