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

## Home reset vs sequential multi-target

Two explicit modes ([spec.md](../spec.md) § Sequential multi-target sequences):

| Mode | Config / CLI | When to use |
|------|--------------|-------------|
| **Sequential multi-target (default)** | `reset_to_home_before_each_trial: false` / `--no-reset-to-home` (CLI default) | Required Spark GUI smoke + operational goal→goal chains |
| **Independent episodes (opt-in)** | `true` / `--reset-to-home` | Dedicated 1.0 rate-gate benchmark from a known start |

Home is always applied **once** at viz session start (first episode). Required GUI smoke **must not** return home between episodes.

```bash
# Default / required GUI smoke (home once only):
./scripts/host/smoke_isaac_viz.sh --gui
# Independent episodes (opt-in rate gate):
./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
```

## Plan recovery (standoff via-waypoints)

When a direct surface plan fails and `plan_recovery_enabled: true` (default), the planner keeps trying until `plan_recovery_timeout_s` (default **90 s**):

1. **Direct** tip → marker surface (capped at `plan_recovery_direct_max_attempts`, default 2)  
2. **Via standoff** candidates from `plan_recovery_standoff_clearances_m` × lateral yaws, ordered **nearest → farthest** from the current tip (progressive distance). Candidates closer than `plan_recovery_min_standoff_travel_m` (default **0.01 m**) are skipped so we do not plan to a near-identical pose. Plan `start → standoff`, then `standoff → surface`.  
3. Optional **partial via1 execution** (Isaac viz): move the EE on a successful via1 even if via2 fails, then continue from the new pose.

The **first** recovery pass always runs after a failed direct plan (so a slow direct `IK_FAIL` cannot skip recovery). Further work respects the timeout.

Yellow + motionless means the wall-clock budget expired with no executable path. Logs must include `via_attempts=N` / `via1_…` / `travel_m=…`.

### Tip-face contact (red → green → success gate)

Green requires the **middle of the EE tip contact pad** on the marker surface along the approach ray, classified by `classify_tip_contact(...)` (which `ee_contacts_target` wraps). It returns `(ok, reason, metrics)` and rejects the failure modes that previously registered green:

- `through` — tip crossed to the far hemisphere along the approach (passed **through** the sphere).
- `side_graze` — contact off the tip-face pad (lateral > `TARGET_MARKER_TIP_FACE_RADIUS_M`).
- `wrong_side_axis` — flange +Z not aligned with the **outward** normal. The check is now **signed** (`TARGET_MARKER_TOOL_AXIS_TOL_RAD` ≈ 35° on `axis_out`, not folded), so it rejects both the **side/barrel** contact (`axis_out ≈ 90°`) and the **flipped/back** contact where +Z points toward the center (`axis_out ≈ 180°`). The flipped case is exactly the "marker turns green but touches the EE from the inside/back" defect the old folded check accepted.

**Tool-axis sign convention (empirical):** commanding the contact pose with +Z **inward** (toward center) makes cuRobo `IK_FAIL` on every via (0 green); the reachable, visually-correct contacts measure `axis_out ≈ 0–7°`. So `build_sphere_contact_approach` commands +Z along the **outward** normal and the gate requires `axis_out` small. Note `contact_geometry`'s +Z is opposite the URDF/FK tool +Z, so the *achieved* front contact reads `axis_out ≈ 0` — confirm against the physical MyCobot flange before hardware.

**Authoritative final-pose check:** the live green during motion is only visual; after the hold the **settled** pose is re-classified. A green that settles wrong-side/through is overridden to `CONTACT_INVALID_SIDE` → `PLAN_FAIL(invalid_side)` — a transient green flash never counts as success.

**Via-pressure (repeated-via math):** `ViaPressureTracker` folds each episode's via count into a via-usage EMA `E_i = α·u_i + (1−α)·E_{i−1}` (`u_i = 1[via≥1]`, `α = 0.5`), a via-count EMA `A_i`, and a consecutive-via streak `S_i`. `VIA_PRESSURE_HIGH` fires when `E_i ≥ 0.6` **and** `S_i ≥ 3`, flagging systematically wrong-sided approaches across consecutive episodes.

The viz logs `MARKER_CONTACT / MARKER_THROUGH / MARKER_SIDE_GRAZE / MARKER_WRONG_SIDE` (with dist/penetration/lateral/`axis_in`/`axis_out`), `CONTACT_INVALID_SIDE`, `VIA_PRESSURE` / `VIA_PRESSURE_HIGH`, plus `EE_CLOSE` and `HIGH_RETRY_WHEN_CLOSE` to attribute high retry counts when the EE starts near the target.

**Planning policy:** tip/flange spheres stay **on** until a short oriented standoff (`contact_standoff_m`); only the final axial standoff→pierce segment omits tip spheres (`try_oriented_tip_face_contact`). The post-via tip-omit cap `contact_via_nudge_max_m` is kept **short** (0.020 m) so a long tip-omit move cannot sweep the EE barrel through the marker; longer gaps are closed by a spheres-ON approach (long tip-omit → `contact_nudge_direct_refused`). This avoids EE-side sweeps through the volumetric marker.

**Contact is required for success:** A trial that gets PLAN_OK but the tip never reaches the sphere surface (`MARKER_NO_CONTACT`) is **reclassified as PLAN_FAIL** — the sphere turns yellow, and the trial counts against the rate gate. This ensures `min_plan_ok_rate: 1.0` means 100% contact, not just 100% planning.

**Overlapping targets skipped:** When `reset_to_home_before_each_trial` is on, targets whose marker sphere overlaps the robot's collision capsules at home are logged as `SKIP_OVERLAPPING_TARGET` and excluded from the rate (they are geometrically invalid — the planner would always reject them with `INVALID_START_STATE_WORLD_COLLISION`).

Headless audit (fails if any PLAN_FAIL has zero via attempts):

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_plan_recovery.sh --num-trials 16
```

Unit tests: `tests/test_recovery_audit.py`.

## Coping with motion-planning / IK failures (strategy)

| Strategy | Status | Notes |
|----------|--------|-------|
| Fail-closed gate | Implemented | Yellow marker; no motion |
| Independent-episode home reset | Optional (YAML default **on** for 1.0 gate) | Not the sequential multi-target path |
| Approach standoff then contact (via) | **Implemented** | Timed recovery; nearest → farthest |
| Multi-seed IK bank (joint space) | **Implemented** | `ik_seed_bank.py`; via2 IK fallback |
| Plan / open-loop to preparatory `q_seed` | **Implemented** | `try_move_to_preparatory_seed` on `INVALID_START` |
| Base→target reposition via (`EE_CLOSE` `IK_FAIL`) | **Implemented** | `try_radial_reposition_via`; valid-start but tip folded inside the standoff shell — back off along the base→target radial to an extended pre-approach, then retry oriented contact. Bounded by `plan_recovery_reposition_max_attempts` |
| Home-blend | Last resort | After seed bank exhausted |
| MoveIt 2 / cuMotion pipelines | Documented | Optional later |

**Do not rely on random Cartesian points farther from the target** as the primary IK recovery. Prefer joint-space seed banks (current / home / prior goals), MoveIt-style multi-attempt reseeding, and IKSel-style “far from failed seeds,” then collision-aware (or open-loop, if start is colliding) motion to a preparatory configuration. See [spec.md](../spec.md) § IK failure → preparatory repositioning.

**Residual learning is not the right tool for this.** Phase 3–4 residuals are bounded `Δq` on top of classical IK. Use planners + classical multi-seed IK for path / seed existence.

### MoveIt 2 vs cuRobo (this context)

For MyCobot + a single volumetric marker in Isaac Sim, **cuRobo is the better default**: GPU MotionGen, already wired, fast retries, mesh-fitted spheres. MoveIt 2 (OMPL) can be stronger for crowded scenes, named states, and ROS 2 hardware stacks with approach/retreat pipelines, but it is not clearly better here and would add a second planning stack without Isaac/cuRobo sphere parity. Prefer finishing cuRobo via-recovery first; evaluate MoveIt / Isaac ROS cuMotion if recovery rates stay low on hardware.

Useful libraries: [cuRobo](https://curobo.org/), [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html), [MoveIt 2](https://moveit.picknik.ai/), [OMPL](https://ompl.kavrakilab.org/).

## Dexterous prescreen, orientation cone, and budget (2026-07-17)

Near-envelope targets often `IK_FAIL` the exact outward-normal contact pose.
Three mechanisms (see [spec.md](../spec.md) § *Dexterous-workspace prescreen …*):

- **Dexterity prescreen** (`planning/dexterity.py`) — deterministic DLS-IK
  reachability of the pad-facing contact pose (pierce on the `base→target` ray,
  a bounded orientation cone, **both** tool-axis signs, FK-sampled orientation-
  aware seeds). Position/reach-unreachable → `SKIPPED_UNREACHABLE`, excluded from
  the gate. Orientation-limited-but-reachable → **not** skipped by default
  (`plan_prescreen_skip_orientation_infeasible: false`): DLS is not a reliable
  6-DOF orientation-completeness oracle, so skipping on it would over-skip and
  dishonestly inflate the gate. A reliable analytic/IKFast/TRAC-IK oracle is the
  prerequisite for enabling orientation skipping (see [REFERENCES.md](../REFERENCES.md)).
- **Bounded orientation cone** (`contact_orientation_cone`) — tries pad-facing
  tilts ≤ `contact_orientation_cone_max_rad` (≈17°, under the 35° gate tol) when
  the exact normal `IK_FAIL`s; the chosen orientation is threaded into the
  tip-omit nudge. Honest by construction — never accepts side/through contacts.
- **Planning budget** — `curobo_max_attempts` 4→6 + `curobo_num_ik_seeds` /
  `curobo_num_trajopt_seeds`. Helps most under GUI GPU contention, where cuRobo
  gets fewer `plan_single` attempts inside the wall-clock recovery budget than
  headless (why the GUI test shows more failures than headless — see STATUS.md).

**Frame-convention reminder (unchanged):** the planner commands tool +Z along the
**outward** normal (empirically the cuRobo-reachable sign); the gate measures the
achieved FK +Z against the outward normal (`axis_out` small). The cone tilts the
commanded direction, so achieved `axis_out` stays within the tilt of the exact
normal — safely under the gate tolerance.

## Honest limits

- Fitted spheres approximate meshes (cuRobo `VOXEL_VOLUME_SAMPLE_SURFACE`); not exact mesh–mesh contact.
- The dexterity prescreen's DLS oracle is **approximate** — it reliably detects
  reach-unreachability, but not orientation-completeness (hence orientation
  skipping is opt-in only).
- `G_base.dae` is authored in mm; the fitter auto-scales by 0.001.
- Tip/flange spheres are kept; the tip plans to the marker **surface** (+ margin) so flange immersion is not forced.
- cuRobo requires host Isaac `python.sh` + CUDA; CI uses NumPy fallback.
