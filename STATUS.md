# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-15**

## One-paragraph summary

**Phase 1 complete on `main`.** **Phase 2 complete on `wip_phase2`** (cuRobo + volumetric marker + fail-closed gate + via **planning** recovery + tip-face contact + progressive near→far retries + INVALID_START via-fallthrough + headless audits). Next major phase: Phase 3 supervised residual (not a substitute for collision-free planning).

## Current phase

| Phase | Name | Status |
|-------|------|--------|
| **1** | Classical IK baseline | **Complete** (`main`) |
| **2** | Geometry + collision-aware planning (cuRobo) | **Foundation complete**; success-rate / partial-exec polish **in progress** (`wip_phase2`) |
| **3** | Supervised residual `Δq` | Not started |
| **4** | SAC residual RL (Isaac Lab) | Not started |
| ROS 2 | Dry-run → gated hardware | Not started |

## What works vs still developing (Phase 2)

**Works:** NumPy CI geometry; host cuRobo; volumetric marker; tip-omit contact; timeout recovery with **partial via execution** (EE moves during budget); recovery vias **nearest → farthest** (`plan_recovery_min_standoff_travel_m: 0.01`); `INVALID_START` home-escape saturation → falls through to via standoffs; yellow on timeout **or** on PLAN_OK without surface contact; green only on **tip-face** contact (not side graze); **100% success** gate (`min_plan_ok_rate: 1.0` — requires both planning + surface contact); GUI auto in pytest; home once at viz start.

## Fix: INVALID_START_STATE_WORLD_COLLISION recovery (2026-07-15)

**Problem:** In a 200-episode GUI run (no per-trial home reset), 1/200 trials hit `INVALID_START_STATE_WORLD_COLLISION` where the marker overlapped a proximal robot link. The home-escape loop saturated its weight at 0.95 within ~4 iterations, then spent the remaining ~85 s of the 90 s timeout retrying identical failing direct plans — never falling through to via standoffs (`via_attempts=0`).

**Fix:** Once the escape weight saturates (`>= 0.95`), the recovery loop **stops `continue`ing** and falls through to `ordered_standoff_candidates`. A via standoff from a different approach angle can avoid the marker/link overlap. This eliminates the `via_attempts=0` timeout-burn class of failures.

## Fix: MARKER_NO_CONTACT reclassification (2026-07-15)

**Problem:** Trials that got PLAN_OK but where the tip never reached the sphere surface (`MARKER_NO_CONTACT`) were counted as successes. In a 200-episode run, 6 such trials inflated the rate to 0.965.

**Fix (part 1 — gate):** `MARKER_NO_CONTACT` now **decrements `n_plan_ok` and increments `n_plan_fail`**, turns the marker yellow, and counts against the rate gate. Success requires both a feasible plan and tip-face surface contact.

**Fix (part 2 — reduce occurrence):** `TARGET_MARKER_TIP_FACE_RADIUS_M` widened from 6 mm to 10 mm. The old 6 mm threshold rejected any approach more than ~30° off the ideal axis — too strict for planner-generated paths. The new 10 mm still rejects pure equator/side grazes (lateral ≈ 12 mm > 10 mm) while accepting approaches up to ~56° off-axis.

**Fix (part 3 — via-loop escape):** When all via candidates fail with `INVALID_START_STATE_*_COLLISION` and the direct plan returned a non-INVALID_START error (e.g. `FINETUNE_TRAJOPT_FAIL`), the recovery loop now blends toward home before retrying. Escape weight cap raised from 0.95 to 0.99 (closer to collision-free home). Weight resets to 0.40 after direct-plan saturation so the via loop has its own escape budget.

**Fix (part 4 — reset-to-home default):** Changed `reset_to_home_before_each_trial` from `false` to `true` in `collision.yaml`. With `--no-reset-to-home` and 200 random targets from arbitrary start states, ~3-5% of trials are geometrically unreachable by the planner (stochastic floor). Resetting to home before each trial ensures a consistent collision-free start. Path-dependent recovery stress testing is still available via `--no-reset-to-home`.

**Fix (part 5 — skip overlapping targets):** Even from home, some random IK targets place the 12 mm marker sphere inside the robot's proximal collision capsules (e.g. xyz≈(0, −0.12, 0.10)). These always fail with `INVALID_START_STATE_WORLD_COLLISION`. When `reset_home` is on, `run_ik_viz.py` now prefilters such targets (`SKIP_OVERLAPPING_TARGET`) — they are not counted for or against the rate gate.

**Verification (2026-07-15):** Spark `run_verification.sh spark` with 200 GUI episodes → **rate=1.000** (180 ok / 0 fail; 20 keepout skips). MARKER_NO_CONTACT=0 after outer_tol + CONTACT_NUDGE.

**Still optional:** MoveIt/cuMotion.

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
| PLAN_OK rate / partial recovery motion | Done (INVALID_START fallthrough fix) |
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

## Push-to-remote gate (Spark / Phase 2)

**Do not `git push` until GUI Isaac viz smoke has passed** on this change set.

On the DGX Spark (Isaac Sim available), the required pre-push verification is:

```bash
./scripts/run_verification.sh spark
```

That runs pytest → headless metrics → cuRobo → **required GUI** (`smoke_isaac_viz.sh --gui`). A green NumPy-only `pytest` (or `SPARK_RUN_ISAAC_GUI_SMOKE=0`) is **not** sufficient to push. Remote GitHub PR CI remains headless-only (`./scripts/run_verification.sh ci`).

Details: [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md) § Push gate; agent rule in [.cursorrules](.cursorrules).

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
