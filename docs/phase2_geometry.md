# Phase 2 — Geometry + Collision-Aware Planning

**Status:** Foundation complete on `wip_phase2` (2026-07-11); PLAN_OK-rate polish still in progress.  
**Briefing:** [phase2_status_and_resume.md](phase2_status_and_resume.md) (what works, WIP, resume after hiatus).  
**Units:** meters, radians, seconds.

## Answer: does planning prevent ground collisions?

**Yes**, when the ground is in the collision world. Phase 1 joint lerp had no geometry, so links could sweep through the floor cube. Phase 2 cuRobo `MotionGen` plans with a ground cuboid (`configs/planning/curobo_world.yaml`); trajectories that penetrate the floor are rejected. Failed plans are gated (no joint motion) when `gate_motion_on_plan_failure: true`.

On the Spark host, a **cuRobo reject is final** (`fallback_numpy_after_curobo_fail: false`). We no longer execute a NumPy lerp after `plan_failed` — that path used coarse capsules, still moved the arm, and could clip the marker while logs looked like a planning failure (`ok_fallback|curobo=plan_failed:...`).

## Volumetric IK target

The red/green marker is a **12 mm sphere with volume**, not a point. Planning keeps mesh-fitted EE/arm collision spheres from intersecting that volume (even when the sphere *center* clears the EE surface). Tip plans to the marker **surface**; tip contact can still turn the marker green.

In Isaac viz the marker relocates only after planning finishes (red on `PLAN_OK`, yellow after recovery fail) — not at trial start. Smoke fails when `PLAN_OK` rate is below `min_plan_ok_rate` (default 0.25). Final surface approaches use a second cuRobo MotionGen with tip/flange spheres omitted so the tip may sit on the marker without `IK_FAIL`.

## What shipped

| Component | Path | Role |
|-----------|------|------|
| NumPy capsules + ground check | `geometry/`, `planning/joint_path.py` | CI / fallback |
| **cuRobo MotionGen** | `planning/curobo_planner.py` | GPU collision-free trajectories (Apache-2.0) |
| **Mesh sphere fitting** | `planning/sphere_fit_mycobot.py` + `fit_mycobot_collision_spheres.sh` | Replace hand-tuned spheres |
| Fitted YAML | `configs/planning/curobo/mycobot_280_collision_spheres.yaml` | Committed fit (meters) |
| Warp 1.15 shim | `_ensure_warp_torch_shim` | Isaac Sim Warp API compat |
| Velocity-fixed URDF | `configs/planning/curobo/mycobot_280_m5_curobo.urdf` | Vendor URDF had `velocity=0` |
| World ground | `configs/planning/curobo_world.yaml` | Floor obstacle |
| Host install / smoke | `scripts/host/install_curobo.sh`, `smoke_phase2_curobo.sh` | Spark GPU |
| Isaac viz | `isaac_sim/run_ik_viz.py` | Plans with volumetric target; `PLAN_OK` / `PLAN_FAIL` |

## How to run

```bash
# CI (NumPy)
./scripts/run_phase2_geometry.sh

# Host: install once, fit spheres (regen), then smoke
./scripts/host/spark_host_exec.sh ./scripts/host/install_curobo.sh
./scripts/host/spark_host_exec.sh ./scripts/host/fit_mycobot_collision_spheres.sh
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase2_curobo.sh

# Full Spark verification (pytest → headless → cuRobo → GUI)
./scripts/run_verification.sh spark
```

## cuRobo marker volume (verified)

Raw `WorldConfig.sphere` entries appear on `world_model` but are **ignored** by the default PRIMITIVE checker. The planner converts them with `WorldConfig.create_obb_world` (sphere → cuboid OBB). Host assert: `./scripts/host/verify_target_obstacle.sh` (SDF cost rises when the marker is at the tip).

## Headless marker↔EE side-contact diagnostic

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_marker_ee_contact.sh --num-trials 24 --seed 0
```

Uses cuRobo kinematics spheres along each planned (or rejected-lerp preview) trajectory and classifies marker intersections as **tip** vs **side**. Faster than watching GUI. See `docs/marker_ee_contact_diag.md`.

Marker colors in GUI: **red** pending, **green** contact, **yellow** plan fail. Plan status is mirrored to Kit **Window → Console** (`carb.log_*`); that is not a full host-terminal tee.

## Home reset before each trial

Optional and **disabled by default** (`reset_to_home_before_each_trial: false`). Enable via YAML or CLI:

```bash
./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
# viz direct:  ... -- --reset-to-home
# diagnose:    ... diagnose_marker_ee_contact.sh --reset-to-home
```

## Plan recovery (standoff via-waypoints)

When a direct surface plan fails and `plan_recovery_enabled: true` (default), the planner keeps trying until `plan_recovery_timeout_s` (default **90 s**):

1. **Direct** tip → marker surface (capped at `plan_recovery_direct_max_attempts`, default 2)  
2. **Via standoff** candidates from `plan_recovery_standoff_clearances_m` × lateral yaws, ordered **nearest → farthest** from the current tip (progressive distance). Candidates closer than `plan_recovery_min_standoff_travel_m` (default **0.01 m**) are skipped so we do not plan to a near-identical pose. Plan `start → standoff`, then `standoff → surface`.  
3. Optional **partial via1 execution** (Isaac viz): move the EE on a successful via1 even if via2 fails, then continue from the new pose.

The **first** recovery pass always runs after a failed direct plan (so a slow direct `IK_FAIL` cannot skip recovery). Further work respects the timeout.

Yellow + motionless means the wall-clock budget expired with no executable path. Logs must include `via_attempts=N` / `via1_…` / `travel_m=…`.

### Tip-face contact (red → green)

Green requires the **middle of the EE tip contact pad** on the marker surface along the approach ray (`ee_contacts_target(..., approach_from_m=…)`). Side / equator grazes are not valid contact.

Headless audit (fails if any PLAN_FAIL has zero via attempts):

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_plan_recovery.sh --num-trials 16
```

Unit tests: `tests/test_recovery_audit.py`.

## Coping with motion-planning failures (strategy)

| Strategy | Status | Notes |
|----------|--------|-------|
| Fail-closed gate | Implemented | Yellow marker; no motion |
| Reset / retract to home | Optional (default **off**) | YAML flag |
| Approach standoff then contact (via) | **Implemented** | Timed recovery loop |
| Lateral / alternate IK seeds | Not yet | Future |
| MoveIt 2 / cuMotion pipelines | Documented | Optional later |

**Residual learning is not the right tool for this.** Phase 3–4 residuals are bounded `Δq` on top of classical IK. Use planners for path existence.

### MoveIt 2 vs cuRobo (this context)

For MyCobot + a single volumetric marker in Isaac Sim, **cuRobo is the better default**: GPU MotionGen, already wired, fast retries, mesh-fitted spheres. MoveIt 2 (OMPL) can be stronger for crowded scenes, named states, and ROS 2 hardware stacks with approach/retreat pipelines, but it is not clearly better here and would add a second planning stack without Isaac/cuRobo sphere parity. Prefer finishing cuRobo via-recovery first; evaluate MoveIt / Isaac ROS cuMotion if recovery rates stay low on hardware.

Useful libraries: [cuRobo](https://curobo.org/), [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html), [MoveIt 2](https://moveit.picknik.ai/), [OMPL](https://ompl.kavrakilab.org/).

## Honest limits

- Fitted spheres approximate meshes (cuRobo `VOXEL_VOLUME_SAMPLE_SURFACE`); not exact mesh–mesh contact.
- `G_base.dae` is authored in mm; the fitter auto-scales by 0.001.
- Tip/flange spheres are kept; the tip plans to the marker **surface** (+ margin) so flange immersion is not forced.
- cuRobo requires host Isaac `python.sh` + CUDA; CI uses NumPy fallback.
