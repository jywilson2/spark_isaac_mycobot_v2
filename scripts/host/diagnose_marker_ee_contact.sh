#!/usr/bin/env bash
# Headless diagnostic: does the volumetric IK marker intersect the EE *side*?
#
# Runs without Isaac Kit GUI. Uses cuRobo kinematics spheres along planned
# trajectories and classifies tip-zone vs side contacts.
#
#   ./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_marker_ee_contact.sh
#   ./scripts/host/diagnose_marker_ee_contact.sh --num-trials 48 --seed 0
#
# Exit 0 if no *executed* path has side contact; exit 1 if any planned path clips.
# Rejected-lerp previews are logged but do not fail the gate (GUI already gates them).
# JSON: assets/logs/marker_ee_contact_diag.json  MD: docs/marker_ee_contact_diag.md
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1

ROOT="${SPARK_REPO_ROOT}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
PY="${ISAACSIM_PYTHON_EXE}"

NUM_TRIALS="${MARKER_DIAG_NUM_TRIALS:-24}"
SEED="${MARKER_DIAG_SEED:-0}"
MAX_WAYPOINTS_LOG="${MARKER_DIAG_MAX_WP_LOG:-8}"
# Pass through extra CLI args to Python.
EXTRA_ARGS=("$@")

echo "=== Headless marker↔EE contact diagnostic ==="
echo "ISAACSIM_PYTHON_EXE=${PY}"
echo "NUM_TRIALS=${NUM_TRIALS} SEED=${SEED}"

"${PY}" - "${NUM_TRIALS}" "${SEED}" "${MAX_WAYPOINTS_LOG}" "${ROOT}" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import torch
from curobo.types.robot import JointState

from isaac_sim.target_marker import TARGET_MARKER_RADIUS_M
from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.kinematics.baseline_eval import evaluate_baseline
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.robot_home import load_home_joint_positions_rad
from residual_adaptive_ik.kinematics.urdf_model import DEFAULT_REVOLUTE_JOINT_NAMES
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    _ensure_warp_torch_shim,
    curobo_available,
    load_planning_config,
)
from residual_adaptive_ik.planning.recovery import plan_collision_free_with_recovery
from residual_adaptive_ik.planning.marker_contact_diag import (
    analyze_waypoint_spheres,
    summarize_trial_waypoints,
)
from residual_adaptive_ik.utils.logging_utils import write_json

num_trials = int(sys.argv[1])
seed = int(sys.argv[2])
max_wp_log = int(sys.argv[3])
repo = Path(sys.argv[4])

# Optional: --num-trials N --seed S / --reset-to-home already partially applied.
argv = sys.argv[5:]
reset_home_cli = None
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
    elif argv[i] == "--fail-on-side":
        i += 1  # default behavior
    elif argv[i] in ("-h", "--help"):
        print(
            "Usage: diagnose_marker_ee_contact.sh "
            "[--num-trials N] [--seed S] [--reset-to-home|--no-reset-to-home]"
        )
        raise SystemExit(0)
    else:
        i += 1

assert curobo_available(), "cuRobo/CUDA required for this diagnostic"
_ensure_warp_torch_shim()

print(f"Evaluating baseline pool for {num_trials} diagnostic trials (seed={seed})...")
_metrics, trials = evaluate_baseline(
    n_poses=max(240, num_trials * 4), seed=seed, return_trials=True
)
chosen_trials = [t for t in trials if t.success][:num_trials]
if len(chosen_trials) < num_trials:
    chosen_trials = trials[:num_trials]

planner = CuRoboMotionPlanner(omit_tip_links=False)
planner._ensure()
mg = planner._motion_gen
device = planner._device
joint_names = list(DEFAULT_REVOLUTE_JOINT_NAMES)
plan_cfg = load_planning_config()
q_home = load_home_joint_positions_rad()
if reset_home_cli is None:
    reset_home = bool(plan_cfg.get("reset_to_home_before_each_trial", False))
else:
    reset_home = bool(reset_home_cli)
print(
    f"reset_to_home_before_each_trial={reset_home} "
    f"plan_recovery_enabled={plan_cfg.get('plan_recovery_enabled', True)} "
    f"timeout_s={plan_cfg.get('plan_recovery_timeout_s', 5.0)}"
)

q_start = q_home.copy()

reports = []
n_side_trials = 0
n_side_on_executed = 0
n_side_on_rejected_preview = 0
n_plan_fail = 0
n_tip_only = 0

for k, trial in enumerate(chosen_trials):
    if reset_home:
        q_start = q_home.copy()
    q_goal = np.asarray(trial.q_sol, dtype=float).reshape(6)
    marker = np.asarray(trial.target.position_m, dtype=float).reshape(3)
    obs = [SphereObstacle(center_m=marker, radius_m=TARGET_MARKER_RADIUS_M, name="ik_target")]
    traj = plan_collision_free_with_recovery(
        q_start, q_goal, prefer_curobo=True, planner=planner, obstacles=obs
    )
    if not traj.ok:
        n_plan_fail += 1
        alphas = np.linspace(0.0, 1.0, 25)
        waypoints = np.stack([(1 - a) * q_start + a * q_goal for a in alphas], axis=0)
        backend = traj.backend
        msg = traj.message
        plan_ok = False
    else:
        waypoints = np.asarray(traj.waypoints_rad, dtype=float)
        backend = traj.backend
        msg = traj.message
        plan_ok = True

    if waypoints.shape[0] > 80:
        idx = np.linspace(0, waypoints.shape[0] - 1, 80).astype(int)
        waypoints = waypoints[idx]

    tip0 = forward_kinematics(q_start).position_m
    approach = marker - tip0
    wp_reports = []
    for wi, q in enumerate(waypoints):
        q = np.asarray(q, dtype=float).reshape(6)
        tip = forward_kinematics(q).position_m
        js = JointState.from_position(
            torch.as_tensor(q.reshape(1, 6), dtype=torch.float32, device=device),
            joint_names=joint_names,
        )
        sph = mg.compute_kinematics(js).robot_spheres[0].detach().cpu().numpy()
        wp_reports.append(
            analyze_waypoint_spheres(
                sph,
                tip,
                marker,
                marker_radius_m=TARGET_MARKER_RADIUS_M,
                waypoint_index=wi,
                approach_m=approach,
            )
        )

    summary = summarize_trial_waypoints(
        wp_reports,
        trial_index=int(trial.index),
        plan_success=plan_ok,
        plan_backend=backend,
        plan_message=msg,
        marker_center_m=marker,
        marker_radius_m=TARGET_MARKER_RADIUS_M,
    )
    slim_wps = []
    for w in summary.waypoints:
        if (
            w.has_side_contact
            or w.n_tip_hits > 0
            or w.waypoint_index % max(1, len(summary.waypoints) // max_wp_log) == 0
        ):
            slim_wps.append(
                {
                    "i": w.waypoint_index,
                    "tip_to_marker_m": w.tip_to_marker_m,
                    "tip_inside": w.tip_inside_marker,
                    "n_tip": w.n_tip_hits,
                    "n_side": w.n_side_hits,
                    "side_hits": [
                        {
                            "si": h.sphere_index,
                            "lateral_m": h.lateral_m,
                            "dist_tip_m": h.dist_to_tip_m,
                            "dist_marker_m": h.dist_to_marker_m,
                            "r_m": h.radius_m,
                        }
                        for h in w.hits
                        if h.kind == "side"
                    ][:6],
                }
            )
    path_kind = (
        f"planned_{backend}" if plan_ok else "rejected_lerp_preview"
    )
    reports.append(
        {
            "trial_index": summary.trial_index,
            "plan_success": summary.plan_success,
            "plan_backend": summary.plan_backend,
            "path_kind": path_kind,
            "plan_message": summary.plan_message[:200],
            "marker_center_m": summary.marker_center_m,
            "n_waypoints": summary.n_waypoints,
            "n_waypoints_with_side": summary.n_waypoints_with_side,
            "n_waypoints_with_tip": summary.n_waypoints_with_tip,
            "first_side_waypoint": summary.first_side_waypoint,
            "max_side_hits_on_waypoint": summary.max_side_hits_on_waypoint,
            "has_side_contact": summary.has_side_contact,
            "waypoints_sample": slim_wps[: max_wp_log + 4],
        }
    )
    side = summary.has_side_contact
    if side:
        n_side_trials += 1
        if plan_ok:
            n_side_on_executed += 1
        else:
            n_side_on_rejected_preview += 1
    elif summary.n_waypoints_with_tip > 0:
        n_tip_only += 1
    flag = "SIDE" if side else ("TIP" if summary.n_waypoints_with_tip else "CLEAR")
    print(
        f"[{k+1}/{len(chosen_trials)}] trial={summary.trial_index} "
        f"path={path_kind} contact={flag} "
        f"side_wps={summary.n_waypoints_with_side} "
        f"first_side={summary.first_side_waypoint}"
    )
    # Path-dependent starts when home reset is off (recovery testing).
    if not reset_home:
        q_start = q_goal.copy()

# Exit fails if an *executed* path (plan_success) has side contact — that is
# what GUI would show. Rejected-path previews are reported but do not fail the
# gate by themselves (motion is gated when planning fails).
out = {
    "num_trials": len(chosen_trials),
    "seed": seed,
    "marker_radius_m": TARGET_MARKER_RADIUS_M,
    "n_trials_with_side_contact": n_side_trials,
    "n_side_on_executed_paths": n_side_on_executed,
    "n_side_on_rejected_lerp_preview": n_side_on_rejected_preview,
    "n_trials_tip_contact_only": n_tip_only,
    "n_plan_fail": n_plan_fail,
    "pass": n_side_on_executed == 0,
    "note": (
        "Side contact on rejected_lerp_preview means a naive joint lerp would "
        "clip the EE; GUI gates those when plan_success=false. "
        "Side contact on planned_* means the executed trajectory still clips."
    ),
    "trials": reports,
}
out_path = repo / "assets" / "logs" / "marker_ee_contact_diag.json"
out_path.parent.mkdir(parents=True, exist_ok=True)
write_json(out_path, out)

md_path = repo / "docs" / "marker_ee_contact_diag.md"
md = [
    "# Marker ↔ EE contact diagnostic (headless)",
    "",
    f"- Trials: **{len(chosen_trials)}** (seed={seed})",
    f"- Marker radius: **{TARGET_MARKER_RADIUS_M} m**",
    f"- Side contact on **executed** paths: **{n_side_on_executed}**",
    f"- Side contact on rejected-lerp *preview*: **{n_side_on_rejected_preview}**",
    f"- Tip-only contact trials: **{n_tip_only}**",
    f"- Plan failures: **{n_plan_fail}**",
    f"- Gate result: **{'PASS' if n_side_on_executed == 0 else 'FAIL'}** "
    f"(fails only when an executed path has side contact)",
    "",
    f"JSON: `{out_path}`",
    "",
    "## Per-trial",
    "",
    "| Trial | Path | Contact | Side WPs | First side WP |",
    "|------:|------|---------|---------:|--------------:|",
]
for r in reports:
    md.append(
        f"| {r['trial_index']} | {r['path_kind']} | "
        f"{'SIDE' if r['has_side_contact'] else 'ok'} | {r['n_waypoints_with_side']} | "
        f"{r['first_side_waypoint']} |"
    )
md_path.write_text("\n".join(md) + "\n", encoding="utf-8")
print(f"JSON: {out_path}")
print(f"Markdown: {md_path}")
print(
    f"SUMMARY executed_side={n_side_on_executed} "
    f"rejected_preview_side={n_side_on_rejected_preview} "
    f"tip_only={n_tip_only} plan_fail={n_plan_fail}"
)
raise SystemExit(0 if n_side_on_executed == 0 else 1)
PY
