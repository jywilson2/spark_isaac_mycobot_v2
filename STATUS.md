# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-12**

## One-paragraph summary

**Phase 1 complete on `main`.** **Phase 2 foundation complete on `wip_phase2`** (cuRobo + volumetric marker + fail-closed gate + via **planning** recovery + tip-face contact + progressive near→far retries + headless audits). **PLAN_OK rate and visible/partial recovery execution are still being improved** — see [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md). Next major phase: Phase 3 supervised residual (not a substitute for collision-free planning).

## Current phase

| Phase | Name | Status |
|-------|------|--------|
| **1** | Classical IK baseline | **Complete** (`main`) |
| **2** | Geometry + collision-aware planning (cuRobo) | **Foundation complete**; success-rate / partial-exec polish **in progress** (`wip_phase2`) |
| **3** | Supervised residual `Δq` | Not started |
| **4** | SAC residual RL (Isaac Lab) | Not started |
| ROS 2 | Dry-run → gated hardware | Not started |

## What works vs still developing (Phase 2)

**Works:** NumPy CI geometry; host cuRobo; volumetric marker; tip-omit contact; timeout recovery with **partial via execution** (EE moves during budget); recovery vias **nearest → farthest** (`plan_recovery_min_standoff_travel_m: 0.01`); yellow only after timeout; green only on **tip-face** contact (not side graze); **100% PLAN_OK** gate (`min_plan_ok_rate: 1.0`, timeout **90 s**); GUI auto in pytest; home once at viz start.

**Still developing:** optional polish on rare missed green; MoveIt/cuMotion; merge to `main`.

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
| Deferred marker + min PLAN_OK rate gate | Done |
| Contact tip-omit + GUI pytest (Spark) | Done |
| PLAN_OK rate / partial recovery motion | Mostly done (rate gate green with home reset); partial-exec polish optional |
| Phase 3 / 4 / hardware | Not started |

## GUI command (watch collision-free arm motion)

```bash
cd /home/jywilson/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
git checkout wip_phase2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
# one-time: ./scripts/host/install_curobo.sh
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui
# optional independent starts (not default GUI smoke):
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
```

Look for `Phase 2 planner: cuRobo MotionGen`, `PLAN_OK` / `PLAN_FAIL` with `via_attempts=N`, marker red→**green** on tip-face contact or yellow on fail. GUI smoke resets home **once** at start only (no per-trial `--reset-to-home`).

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
