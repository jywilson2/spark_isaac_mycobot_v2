#!/usr/bin/env bash
# Fit cuRobo collision spheres to MyCobot 280 meshes (host Isaac python + CUDA).
#
# Writes configs/planning/curobo/mycobot_280_collision_spheres.yaml
# See residual_adaptive_ik.planning.sphere_fit_mycobot and spec.md Phase 2.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.isaac_host.sh
source "${SCRIPT_DIR}/env.isaac_host.sh"
spark_host_apply_env || exit 1

ROOT="${SPARK_REPO_ROOT}"
export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
PY="${ISAACSIM_PYTHON_EXE}"

echo "=== Fit MyCobot collision spheres (cuRobo Mesh.get_bounding_spheres) ==="
"${PY}" - <<'PY'
from residual_adaptive_ik.planning.sphere_fit_mycobot import (
    adjacent_self_collision_ignore,
    fit_link_collision_spheres,
    write_collision_spheres_yaml,
)

n = 16
r_surf = 0.005
spheres = fit_link_collision_spheres(
    n_spheres_per_link=n,
    surface_sphere_radius_m=r_surf,
)
path = write_collision_spheres_yaml(
    spheres,
    self_collision_ignore=adjacent_self_collision_ignore(),
    n_spheres_per_link=n,
    surface_sphere_radius_m=r_surf,
)
total = sum(len(v) for v in spheres.values())
print(f"Wrote {path} links={len(spheres)} spheres={total}")
for link, lst in spheres.items():
    radii = [s["radius"] for s in lst]
    print(
        f"  {link}: n={len(lst)} r_min={min(radii):.4f} r_max={max(radii):.4f} m"
    )
print("fit_mycobot_collision_spheres DONE")
PY
