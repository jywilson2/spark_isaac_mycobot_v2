# Phase 2 — Geometry + Collision-Aware Planning

**Status:** Foundation landed on branch `wip_phase2` (2026-07-11).  
**Units:** meters, radians.

## What shipped

| Component | Path | Role |
|-----------|------|------|
| Capsule / sphere primitives | `src/residual_adaptive_ik/geometry/` | Approximate link volumes vs obstacles |
| Collision-checked joint lerp | `src/residual_adaptive_ik/planning/` | Sample Phase-1-style joint paths |
| Config | `configs/planning/collision.yaml` | Link radius, sample count |
| Validation hook | `validate_solution(..., obstacles=)` | Geometry-backed `"collision"` reject |
| CI entry | `scripts/run_phase2_geometry.sh` | pytest + NumPy path smoke |
| Host viz logging | `isaac_sim/run_phase1_ik_viz.py` | Prints `PATH_OK` / `PATH_COLLISION` |

## Library choice

Phase 2 CI uses **NumPy-only** capsule–sphere checks so container / GitHub CI stays kit-free.

Open-source NVIDIA / ROS options documented for later enrichment (do not replace residual IK):

- **cuRobo** (Apache-2.0) — preferred GPU trajectory backend on DGX Spark
- **Isaac Sim PhysX** — contact queries during Kit runs
- **MoveIt 2** — ROS 2 hardware planning stacks

## How to run

```bash
# CI / container
./scripts/run_phase2_geometry.sh

# Full verification on Spark (includes Isaac GUI after headless)
./scripts/run_verification.sh spark
```

## Honest limits

- Capsules are coarse (not mesh-accurate FCL).
- Path layer is a **checked joint lerp**, not a full RRT/cuRobo planner yet.
- Viz still executes the lerp for visualization; collision results are logged for metrics and future gating.

## Next Phase 2 milestones

1. Gate motion / reject trials when `PATH_COLLISION` and an alternate IK seed exists.
2. Optional cuRobo backend behind a feature flag on the host.
3. Self-collision (non-adjacent capsules).
