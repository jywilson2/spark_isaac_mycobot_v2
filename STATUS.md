# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

**Phase 1 complete on `main`.** **Phase 2 foundation complete on `wip_phase2`** (cuRobo + volumetric marker + fail-closed gate + via **planning** recovery + headless audits). **PLAN_OK rate and visible/partial recovery execution are still being improved** — see [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md). Next major phase: Phase 3 supervised residual (not a substitute for collision-free planning).

## Current phase

| Phase | Name | Status |
|-------|------|--------|
| **1** | Classical IK baseline | **Complete** (`main`) |
| **2** | Geometry + collision-aware planning (cuRobo) | **Foundation complete**; success-rate / partial-exec polish **in progress** (`wip_phase2`) |
| **3** | Supervised residual `Δq` | Not started |
| **4** | SAC residual RL (Isaac Lab) | Not started |
| ROS 2 | Dry-run → gated hardware | Not started |

## What works vs still developing (Phase 2)

**Works:** NumPy CI geometry; host cuRobo MotionGen; mesh-fitted spheres; volumetric marker (OBB); tip surface approach; fail-closed (yellow, no unsafe lerp); standoff via **planning** recovery + `diagnose_plan_recovery.sh`; marker side-contact diagnostic; `smoke_isaac_viz.sh` / `run_isaac_viz.sh`.

**Still developing:** higher PLAN_OK rate; optional execute-to-standoff when via1 OK / via2 fails; lateral/IK-seed retries; MoveIt/cuMotion; merge to `main`.

Full table + **resume-after-hiatus steps:** [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md).

## Checklist

| Item | Status |
|------|--------|
| Phase 1 FK / DLS / Isaac viz | Done |
| Four-phase renumber | Done |
| Phase 2 NumPy geometry + ground | Done |
| Phase 2 cuRobo MotionGen + host smoke | Done |
| Volumetric marker + OBB + fail-closed | Done |
| Via planning recovery + headless audit | Done |
| PLAN_OK rate / partial recovery motion | In progress |
| Phase 3 / 4 / hardware | Not started |

## GUI command (watch collision-free arm motion)

```bash
cd /home/jywilson/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
git checkout wip_phase2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
# one-time: ./scripts/host/install_curobo.sh
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui
# optional independent starts:
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
```

Look for `Phase 2 planner: cuRobo MotionGen`, `PLAN_OK` / `PLAN_FAIL` with `via_attempts=N`, marker red→green or yellow.

## Resume after a long break

1. `git checkout wip_phase2 && git pull --rebase origin wip_phase2`
2. Read [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md)
3. `PYTHONPATH=src:. python3 -m pytest tests -q`
4. `./scripts/run_verification.sh spark` (do not pipe through `head`/`tail`)

## Suggested next steps

1. Improve PLAN_OK under volumetric marker; consider partial via execution for GUI visibility.
2. Merge `wip_phase2` → `main` when verification is agreed.
3. Start Phase 3 supervised residual under simulated mismatch.
4. Keep hardware dry-run until `ENABLE_MYCOBOT_HARDWARE_TESTS=1`.

## Related docs

- [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md) · [docs/phase2_geometry.md](docs/phase2_geometry.md) · [README.md](README.md) · [spec.md](spec.md)
