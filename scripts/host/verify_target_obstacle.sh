#!/usr/bin/env bash
# Verify the volumetric IK marker is actually queried by cuRobo's GPU checker.
#
# Historical bug: WorldConfig.sphere updated world_model but the default
# PRIMITIVE checker only loads cuboids — SDF cost stayed 0. Fix:
# WorldConfig.create_obb_world() in CuRoboMotionPlanner._world_config_for_checker.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
ROOT="${SPARK_REPO_ROOT}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

echo "=== Verify cuRobo processes volumetric IK target ==="
"${ISAACSIM_PYTHON_EXE}" - <<'PY'
import numpy as np
import torch
from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState
from curobo.geom.sdf.world import CollisionQueryBuffer

from residual_adaptive_ik.geometry.collision import SphereObstacle
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import DEFAULT_REVOLUTE_JOINT_NAMES
from residual_adaptive_ik.planning.curobo_planner import (
    CuRoboMotionPlanner,
    _ensure_warp_torch_shim,
    curobo_available,
)

assert curobo_available(), "cuRobo/CUDA required"
_ensure_warp_torch_shim()
ta = TensorDeviceType()

lo, hi = load_joint_limits_rad()
q = 0.5 * (lo + hi)
tip = forward_kinematics(q).position_m

planner = CuRoboMotionPlanner(omit_tip_links=False)
planner._ensure()
mg = planner._motion_gen

obs = [SphereObstacle(center_m=tip, radius_m=0.012, name="ik_target")]
planner._apply_obstacles(obs)

# After OBB conversion, the GPU world must list ik_target as a cuboid.
cuboids = {c.name: c for c in (mg.world_model.cuboid or [])}
assert "ik_target" in cuboids, f"ik_target missing from cuboids: {list(cuboids)}"
print(
    "WORLD_CUBOID ik_target dims=",
    list(map(float, cuboids["ik_target"].dims)),
    "pose_xyz=",
    list(map(float, cuboids["ik_target"].pose[:3])),
)

js = JointState.from_position(
    torch.as_tensor(q.reshape(1, 6), dtype=torch.float32, device=ta.device),
    joint_names=list(DEFAULT_REVOLUTE_JOINT_NAMES),
)
sph = mg.compute_kinematics(js).robot_spheres
query = sph.view(1, 1, -1, 4)
wcol = mg.world_coll_checker
buf = CollisionQueryBuffer.initialize_from_shape(query.shape, ta, wcol.collision_types)
weight = torch.ones(1, device=ta.device)
act = torch.zeros(1, device=ta.device)
dist_near = wcol.get_sphere_distance(query, buf, weight, act)

planner._apply_obstacles(
    [SphereObstacle(center_m=np.array([0.0, 0.0, 0.85]), radius_m=0.012)]
)
buf2 = CollisionQueryBuffer.initialize_from_shape(query.shape, ta, wcol.collision_types)
dist_far = wcol.get_sphere_distance(query, buf2, weight, act)

near_sum = float(dist_near.sum().item())
far_sum = float(dist_far.sum().item())
print("SDF_NEAR_sum", near_sum, "SDF_FAR_sum", far_sum)
assert near_sum > far_sum + 1e-6, (
    "cuRobo SDF did not increase with marker at tip — volume not processed"
)
print("VERIFY_TARGET_OBSTACLE_OK")
PY
