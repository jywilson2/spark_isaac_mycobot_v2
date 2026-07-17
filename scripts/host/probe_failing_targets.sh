#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1
cd "${SPARK_REPO_ROOT}"
export PYTHONPATH="${SPARK_REPO_ROOT}/src:${SPARK_REPO_ROOT}"
"${ISAACSIM_PATH}/python.sh" - <<"PY"
import numpy as np
from residual_adaptive_ik.planning.dexterity import contact_pose_is_dexterous, is_in_dexterous_region, load_reach_envelope_m, target_radial_distance_m
from residual_adaptive_ik.planning.pinocchio_ik import pinocchio_available
print("pinocchio", pinocchio_available())
for t in [(0.036,-0.237,0.080),(0.114,-0.200,0.152),(0.136,0.133,0.095)]:
  r=target_radial_distance_m(t); lo,hi=load_reach_envelope_m(); ir,_=is_in_dexterous_region(t,min_reach_m=lo,max_reach_m=hi,margin_m=0.02)
  res=contact_pose_is_dexterous(t,0.02,n_seeds=12,backend="pinocchio")
  print(t, f"radial={r:.3f}", f"in_region={ir}", res.classification, f"feasible={res.feasible}", res.backend, res.note)
PY
