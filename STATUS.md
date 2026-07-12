# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

**Phase 1 complete on `main`.** **Phase 2 complete on `wip_phase2`:** NumPy geometry + ground checks for CI, and **NVIDIA cuRobo** (Apache-2.0) `MotionGen` for collision-free trajectories on the DGX Spark (including a ground cuboid so the arm cannot plan through the floor). Isaac viz executes planned trajectories and gates motion on plan failure. Next: Phase 3 supervised residual.

## Current phase

| Phase | Name | Status |
|-------|------|--------|
| **1** | Classical IK baseline | **Complete** (`main`) |
| **2** | Geometry + collision-aware planning (cuRobo) | **Complete** (`wip_phase2`) |
| **3** | Supervised residual `Δq` | Not started |
| **4** | SAC residual RL (Isaac Lab) | Not started |
| ROS 2 | Dry-run → gated hardware | Not started |

## Checklist

| Item | Status |
|------|--------|
| Phase 1 FK / DLS / Isaac viz | Done |
| Four-phase renumber | Done |
| Phase 2 NumPy geometry + ground | Done |
| Phase 2 cuRobo MotionGen + host smoke | Done |
| Warp 1.15 ↔ cuRobo shim | Done |
| Gate viz motion on plan failure | Done |
| Phase 3 / 4 / hardware | Not started |

## GUI command (watch collision-free arm motion)

```bash
cd /home/jywilson/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
git checkout wip_phase2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
# one-time: ./scripts/host/install_curobo.sh
./scripts/host/run_phase1_isaac.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4
```

From the container:

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh --gui
```

Look for `Phase 2 planner: cuRobo MotionGen` and per-trial `PLAN_OK` / `PLAN_FAIL`.

## Suggested next steps

1. Merge `wip_phase2` → `main` when ready.
2. Start Phase 3 supervised residual under simulated mismatch.
3. Keep hardware dry-run until `ENABLE_MYCOBOT_HARDWARE_TESTS=1`.

## Related docs

- [docs/phase2_geometry.md](docs/phase2_geometry.md) · [README.md](README.md) · [spec.md](spec.md) · [docs/phase1_baseline.md](docs/phase1_baseline.md)
