#!/usr/bin/env bash
# Host Phase 2 cuRobo MotionGen smoke (requires CUDA + install_curobo.sh).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1

ROOT="${SPARK_REPO_ROOT}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
PY="${ISAACSIM_PYTHON_EXE}"

echo "=== Phase 2 cuRobo MotionGen smoke ==="
"${PY}" - <<'PY'
import numpy as np
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    curobo_available,
    plan_collision_free,
)

assert curobo_available(), "cuRobo/CUDA required for this smoke"
lo, hi = load_joint_limits_rad()
# Stay well inside limits (cuRobo clips URDF limits slightly).
q0 = 0.5 * (lo + hi)
q1 = np.clip(q0 + np.array([0.4, -0.3, 0.35, 0.2, -0.25, 0.15]), lo + 0.05, hi - 0.05)
planner = CuRoboMotionPlanner(omit_tip_links=False)
traj = planner.plan_to_joint_goal(q0, q1, max_attempts=8)
print("direct", traj.success, traj.backend, traj.message, "T", traj.waypoints_rad.shape)
if not traj.success:
    raise SystemExit(f"cuRobo direct plan failed: {traj.message}")
assert traj.backend == "curobo", traj
traj2 = plan_collision_free(q0, q1, prefer_curobo=True, planner=planner)
print("wrapper", traj2.success, traj2.backend, traj2.message, "T", traj2.waypoints_rad.shape)
if not traj2.success or traj2.backend != "curobo":
    raise SystemExit(f"cuRobo wrapper failed: {traj2}")

# Volumetric marker processing is asserted by verify_target_obstacle.sh
# (SDF cost rises when the 12 mm OBB is at the tip). Keep this smoke focused
# on MotionGen reachability without a tip-centered obstacle.
print("Phase 2 cuRobo smoke PASSED")
PY
