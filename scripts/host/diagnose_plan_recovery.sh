#!/usr/bin/env bash
# Headless audit: PLAN_FAIL must show via-recovery attempts when recovery is on.
#
# Detects the GUI failure mode "yellow target moves, arm motionless, no recovery"
# at the planning layer (via1_ missing from fail messages).
#
#   ./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_plan_recovery.sh
#   ./scripts/host/diagnose_plan_recovery.sh --num-trials 16 --seed 0
#
# Exit 1 if any failed plan (with marker) has zero via attempts.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1

ROOT="${SPARK_REPO_ROOT}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
PY="${ISAACSIM_PYTHON_EXE}"

NUM_TRIALS="${PLAN_RECOVERY_DIAG_NUM_TRIALS:-16}"
SEED="${PLAN_RECOVERY_DIAG_SEED:-0}"
EXTRA_ARGS=("$@")

echo "=== Headless plan-recovery audit ==="
echo "ISAACSIM_PYTHON_EXE=${PY}"

"${PY}" - "${NUM_TRIALS}" "${SEED}" "${ROOT}" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}" <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from isaac_sim.target_marker import TARGET_MARKER_RADIUS_M
from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.kinematics.baseline_eval import evaluate_baseline
from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    _ensure_warp_torch_shim,
    curobo_available,
    load_planning_config,
)
from residual_adaptive_ik.planning.recovery import (
    plan_collision_free_with_recovery,
    recovery_audit_plan_fail,
    recovery_via_attempts_in_message,
)
from residual_adaptive_ik.utils.logging_utils import write_json

num_trials = int(sys.argv[1])
seed = int(sys.argv[2])
repo = Path(sys.argv[3])
reset_home_cli = None
argv = sys.argv[4:]
i = 0
while i < len(argv):
    if argv[i] == "--num-trials" and i + 1 < len(argv):
        num_trials = int(argv[i + 1])
        i += 2
    elif argv[i] == "--seed" and i + 1 < len(argv):
        seed = int(argv[i + 1])
        i += 2
    elif argv[i] == "--reset-to-home":
        reset_home_cli = True
        i += 1
    elif argv[i] == "--no-reset-to-home":
        reset_home_cli = False
        i += 1
    elif argv[i] in ("-h", "--help"):
        print(
            "Usage: diagnose_plan_recovery.sh "
            "[--num-trials N] [--seed S] [--reset-to-home|--no-reset-to-home]"
        )
        raise SystemExit(0)
    else:
        i += 1

assert curobo_available(), "cuRobo/CUDA required"
_ensure_warp_torch_shim()
cfg = load_planning_config()
recover_on = bool(cfg.get("plan_recovery_enabled", True))
assert recover_on, "plan_recovery_enabled must be true for this audit"

print(f"Evaluating baseline pool (seed={seed})...")
_metrics, trials = evaluate_baseline(
    n_poses=max(240, num_trials * 4), seed=seed, return_trials=True
)
chosen = [t for t in trials if t.success][:num_trials]
if len(chosen) < num_trials:
    chosen = trials[:num_trials]

planner = CuRoboMotionPlanner(omit_tip_links=False)
planner._ensure()
q_home = load_home_joint_positions_rad()
if reset_home_cli is None:
    reset_home = bool(cfg.get("reset_to_home_before_each_trial", False))
else:
    reset_home = bool(reset_home_cli)

q_start = q_home.copy()
reports = []
n_ok = 0
n_fail = 0
n_fail_no_via = 0
n_via_total = 0

for k, trial in enumerate(chosen):
    if reset_home:
        q_start = q_home.copy()
    q_goal = np.asarray(trial.q_sol, dtype=float).reshape(6)
    marker = np.asarray(trial.target.position_m, dtype=float).reshape(3)
    obs = [SphereObstacle(center_m=marker, radius_m=TARGET_MARKER_RADIUS_M, name="ik_target")]
    traj = plan_collision_free_with_recovery(
        q_start, q_goal, prefer_curobo=True, planner=planner, obstacles=obs
    )
    vias = recovery_via_attempts_in_message(traj.message)
    n_via_total += vias
    row = {
        "trial_index": int(trial.index),
        "plan_ok": bool(traj.ok),
        "backend": traj.backend,
        "message": traj.message[:400],
        "via_attempts": vias,
    }
    if traj.ok:
        n_ok += 1
        flag = "OK"
    else:
        n_fail += 1
        audit = recovery_audit_plan_fail(
            traj.message, recovery_enabled=True, had_marker=True
        )
        row["audit_ok"] = bool(audit["ok"])
        row["audit_reason"] = str(audit["reason"])
        if not audit["ok"]:
            n_fail_no_via += 1
            flag = "FAIL_NO_VIA"
        else:
            flag = "FAIL_WITH_VIA"
    reports.append(row)
    print(f"[{k+1}/{len(chosen)}] trial={trial.index} {flag} vias={vias} msg={traj.message[:120]}")
    if not reset_home:
        q_start = q_goal.copy()

out = {
    "num_trials": len(chosen),
    "seed": seed,
    "reset_to_home": reset_home,
    "n_plan_ok": n_ok,
    "n_plan_fail": n_fail,
    "n_plan_fail_without_via": n_fail_no_via,
    "n_via_attempts_total": n_via_total,
    "pass": n_fail_no_via == 0,
    "note": (
        "Fail means a PLAN_FAIL had no via1_ recovery attempt (yellow/motionless "
        "GUI mode with recovery never tried). Via attempts that still fail are OK."
    ),
    "trials": reports,
}
out_path = repo / "assets" / "logs" / "plan_recovery_diag.json"
out_path.parent.mkdir(parents=True, exist_ok=True)
write_json(out_path, out)

md_path = repo / "docs" / "plan_recovery_diag.md"
md = [
    "# Plan recovery audit (headless)",
    "",
    f"- Trials: **{len(chosen)}** (seed={seed})",
    f"- Plan OK: **{n_ok}**",
    f"- Plan FAIL: **{n_fail}**",
    f"- FAIL with **no via** attempt: **{n_fail_no_via}**",
    f"- Total via1 attempts: **{n_via_total}**",
    f"- Result: **{'PASS' if n_fail_no_via == 0 else 'FAIL'}**",
    "",
    f"JSON: `{out_path}`",
    "",
]
md_path.write_text("\n".join(md) + "\n", encoding="utf-8")
print(f"JSON: {out_path}")
print(f"Markdown: {md_path}")
print(
    f"SUMMARY ok={n_ok} fail={n_fail} fail_no_via={n_fail_no_via} "
    f"via_total={n_via_total}"
)
raise SystemExit(0 if n_fail_no_via == 0 else 1)
PY
