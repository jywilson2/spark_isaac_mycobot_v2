#!/usr/bin/env bash
# Phase 2 — geometry + collision-checked joint planning (CI-friendly).
#
# Runs unit tests and a short NumPy path-check demo. No Isaac Kit required.
# Host Isaac GUI still uses scripts/host/run_isaac_viz.sh (path checks log
# inside Kit when enabled). See spec.md Phase 2.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export PYTHONPATH="${ROOT}/src:${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
PY="${PYTHON:-python3}"
if ! command -v "${PY}" >/dev/null 2>&1; then
  PY=python
fi

echo "=== Phase 2 geometry / planning tests ==="
"${PY}" -m pytest \
  tests/test_phase2_geometry.py \
  tests/test_ik_validation.py \
  -q

echo "=== Phase 2 path-check smoke (NumPy) ==="
"${PY}" - <<'PY'
from pathlib import Path
import numpy as np
from residual_adaptive_ik.geometry import SphereObstacle
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.kinematics.urdf_model import load_urdf_model
from residual_adaptive_ik.planning import plan_joint_lerp_checked
from residual_adaptive_ik.utils.logging_utils import write_json

model = load_urdf_model()
lo, hi = load_joint_limits_rad()
rng = np.random.default_rng(0)
q0 = rng.uniform(lo, hi)
q1 = rng.uniform(lo, hi)
obs = [SphereObstacle(center_m=[0.18, 0.0, 0.14], radius_m=0.012, name="ik_target")]
path = plan_joint_lerp_checked(
    q0, q1, obs, n_samples=24, model=model, ignore_tip_segment=True
)
out = Path("/tmp/phase2_path_smoke.json")
write_json(
    out,
    {
        "collides": path.collision.collides,
        "n_waypoints": int(path.waypoints_rad.shape[0]),
        "n_reasons": len(path.collision.reasons),
        "reasons_head": list(path.collision.reasons[:8]),
    },
)
print(f"Wrote {out} collides={path.collision.collides}")
PY

echo "Phase 2 geometry entry complete. See docs/phase2_geometry.md"
