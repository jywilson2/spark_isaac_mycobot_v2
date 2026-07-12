# Phase 2 — Geometry + Collision-Aware Planning

**Status:** Complete on branch `wip_phase2` (2026-07-11).  
**Units:** meters, radians, seconds.

## Answer: does planning prevent ground collisions?

**Yes**, when the ground is in the collision world. Phase 1 joint lerp had no geometry, so links could sweep through the floor cube. Phase 2 cuRobo `MotionGen` plans with a ground cuboid (`configs/planning/curobo_world.yaml`); trajectories that penetrate the floor are rejected. Failed plans are gated (no naive lerp through the floor) when `gate_motion_on_plan_failure: true`.

## What shipped

| Component | Path | Role |
|-----------|------|------|
| NumPy capsules + ground check | `geometry/`, `planning/joint_path.py` | CI / fallback |
| **cuRobo MotionGen** | `planning/curobo_planner.py` | GPU collision-free trajectories (Apache-2.0) |
| Warp 1.15 shim | `_ensure_warp_torch_shim` | Isaac Sim Warp API compat |
| Velocity-fixed URDF | `assets/urdf/mycobot_280_m5_curobo.urdf` | Vendor URDF had `velocity=0` |
| World ground | `configs/planning/curobo_world.yaml` | Floor obstacle |
| Host install / smoke | `scripts/host/install_curobo.sh`, `smoke_phase2_curobo.sh` | Spark GPU |
| Isaac viz | `isaac_sim/run_phase1_ik_viz.py` | Executes planned traj; `PLAN_OK` / `PLAN_FAIL` |

## How to run

```bash
# CI (NumPy)
./scripts/run_phase2_geometry.sh

# Host: install once, then smoke
./scripts/host/spark_host_exec.sh ./scripts/host/install_curobo.sh
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase2_curobo.sh

# Full Spark verification (pytest → headless → cuRobo → GUI)
./scripts/run_verification.sh spark
```

## Honest limits

- Collision spheres are coarse (not mesh-fitted).
- cuRobo requires host Isaac `python.sh` + CUDA; CI uses NumPy fallback.
- Target marker remains visual-only in USD; planning treats it as a sphere obstacle optionally.
