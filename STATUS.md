# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

**Phase 1 (Classical IK Baseline) is complete and verified** on the DGX Spark host: NumPy FK / DLS IK / validation, ≥1000-pose stratified workspace metrics, host Isaac Sim metrics+viz (red→green target sphere on EE contact, joint motion ≤160 °/s), and unified verification (`./scripts/run_verification.sh ci|spark`). Remote: `git@github.com:jywilson2/spark_isaac_mycobot_v2.git`. **Next:** adopt a **four-phase** plan — Phase 2 geometry + collision-aware planning, then supervised residual (Phase 3) and SAC residual (Phase 4).

## Current phase

| Phase | Name | Status |
|-------|------|--------|
| **1** | Classical IK baseline (FK, DLS, validation, Isaac viz) | **Complete** |
| **2** | Geometry + collision-aware planning | **Next** (branch `wip_phase2`) |
| **3** | Supervised residual `Δq` | Not started (was former Phase 2) |
| **4** | SAC residual RL (Isaac Lab) | Not started (was former Phase 3) |
| ROS 2 | Dry-run node → gated hardware | Not started |

## Checklist

| Item | Status |
|------|--------|
| Repo layout / configs / stubs | Done |
| URDF FK + DLS IK + validation | Done |
| Phase 1 baseline ≥1000 poses | Done (`docs/phase1_baseline.md`) |
| Even workspace sampling (12×4×5 bins) | Done |
| Host Isaac Sim Phase 1 viz | Done (`scripts/host/run_phase1_isaac.sh`) |
| Target sphere red→green on contact | Done |
| Host Isaac smoke (TDD) | Done — `./scripts/run_verification.sh spark` |
| Expected Kit warnings catalogued | Done — README § Expected Isaac Sim launch warnings |
| Phase 2 geometry / planning | In progress on `wip_phase2` |
| Phase 3 / 4 / hardware | Not started |

## How to open in Cursor

**File → Open Workspace from File…** → `spark_isaac_mycobot_v2.code-workspace`

## Environment notes

**Container (metrics / unit tests):**

```bash
source scripts/source_container_env.sh
./scripts/download_mycobot_ros2.sh
PYTHONPATH=src:. pytest tests -q
bash scripts/run_phase1_baseline.sh
./scripts/run_verification.sh ci
```

**Host (Isaac Sim GUI — watch the arm move):**

```bash
cd /home/jywilson/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
./scripts/host/run_phase1_isaac.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4
```

Or from the Isaac ROS / Cursor container (delegates to host user + DISPLAY):

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh --gui
```

Target sphere: **red until EE tip contact**, then **green**.

## Suggested next steps

1. Land Phase 1 on `main` (this push).
2. On `wip_phase2`: four-phase renumber + geometry/planning foundation.
3. Keep hardware dry-run until `ENABLE_MYCOBOT_HARDWARE_TESTS=1`.
4. Do not claim sub-mm real-world accuracy without hardware measurements.

## Related docs

- [README.md](README.md) · [spec.md](spec.md) · [CHANGES.md](CHANGES.md) · [docs/phase1_baseline.md](docs/phase1_baseline.md) · [docs/isaac_sim_host.md](docs/isaac_sim_host.md) · [docs/last_prompt.md](docs/last_prompt.md)
