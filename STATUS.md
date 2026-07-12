# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

**Phase 1 is complete on `main`.** Branch **`wip_phase2`** adopts the **four-phase** plan and lands the Phase 2 foundation: NumPy capsule–sphere geometry, collision-checked joint lerps, `validate_solution(..., obstacles=)`, CI entry `./scripts/run_phase2_geometry.sh`, and Isaac viz `PATH_OK` / `PATH_COLLISION` logging. Supervised residual is now **Phase 3**; SAC is **Phase 4**. Remote: `git@github.com:jywilson2/spark_isaac_mycobot_v2.git`.

## Current phase

| Phase | Name | Status |
|-------|------|--------|
| **1** | Classical IK baseline | **Complete** (on `main`) |
| **2** | Geometry + collision-aware planning | **In progress** (`wip_phase2` foundation) |
| **3** | Supervised residual `Δq` | Not started |
| **4** | SAC residual RL (Isaac Lab) | Not started |
| ROS 2 | Dry-run → gated hardware | Not started |

## Checklist

| Item | Status |
|------|--------|
| Phase 1 FK / DLS / validation / Isaac viz | Done |
| Four-phase renumber in spec / scripts / STATUS | Done on `wip_phase2` |
| Phase 2 NumPy geometry + path checks | Done (foundation) |
| Phase 2 cuRobo / PhysX / MoveIt backends | Not started (documented) |
| Phase 3 / 4 / hardware | Not started |

## GUI command (watch the arm move)

```bash
cd /home/jywilson/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
./scripts/host/run_phase1_isaac.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4
```

From the Cursor / Isaac ROS container:

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh --gui
```

Look for per-trial `PATH_OK` or `PATH_COLLISION` lines (Phase 2 logging) and the target sphere turning **green** on EE tip contact.

## Suggested next steps

1. Finish Phase 2: gate motion on path collision; optional cuRobo backend.
2. Start Phase 3 supervised residual under simulated mismatch.
3. Keep hardware dry-run until `ENABLE_MYCOBOT_HARDWARE_TESTS=1`.

## Related docs

- [README.md](README.md) · [spec.md](spec.md) · [docs/phase1_baseline.md](docs/phase1_baseline.md) · [docs/phase2_geometry.md](docs/phase2_geometry.md) · [docs/isaac_sim_host.md](docs/isaac_sim_host.md) · [docs/last_prompt.md](docs/last_prompt.md)
