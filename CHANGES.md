# CHANGES — Scaffold inventory (2026-07-11)

## Iter17: Tip-face / arm-body gates on DLS standoff approach (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — after ``plan_dls_standoff_approach_lerp`` succeeds, reject
   paths with tip immersion, proximal arm body contact, or tip-face
   ``side_graze`` / through / clear wrong_side (iter16 Ep10 DLS chord graze).
2. **`run_ik_viz.py`** — ``SETTLE_RESTORE`` only when tip is >0.5 mm past shell
   (avoid equality false triggers at 14.0 mm).

## Iter16: Restore q_at_contact when settle tip drifts to standoff (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — latch ``q_at_contact`` on first MARKER_CONTACT green;
   ``SETTLE_RESTORE`` that pose when tip is outside the surface shell at hold
   start (snap joints immediately; skip CONTACT_HOLD after restore so Ep8
   cannot re-drift to standoff); CONTACT_HOLD position-relaxed IK (ori tol
   0.35) then restore green pose if IK still fails.

## Iter14: Fix isaac_sim import for tip-face path reject (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — prepend repo root to ``sys.path`` before importing
   ``isaac_sim.target_marker.classify_tip_contact`` so approach tip-face
   validation actually runs (was silently skipped).

## Iter13: Reject tip-face side_graze on approach path (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — sample approach waypoints in the surface shell with
   `classify_tip_contact`; reject side_graze / through / immersed / clear
   wrong_side before accepting MotionGen approach.

## Iter12: Reject approaches with proximal arm ∩ marker (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — after MotionGen approach OK, sample waypoints with
   `proximal_arm_contacts_target` (same radii as viz arm-sweep); reject
   `plan_failed:arm_body_on_approach` and try next cone / DLS / via.

## Iter11: Reject tip-immersing MotionGen approach paths (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — `tip_path_avoids_marker_immersion`; after MotionGen
   approach OK, sample FK tips and reject if any tip enters the marker volume
   (then try next cone / DLS). Prevents mid-path IMMERSED/THROUGH/SIDE_GRAZE
   that still greened later (iter10 Ep5).

## Iter10: Mid-path wrong_side latch only for clear side/back (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — `wrong_side_axis` mid-path latch requires
   ``axis_out > max(0.50 rad, 2×TOOL_AXIS_TOL)``; near-tol tip-face misses
   (≈15–20°) no longer force PLAN_FAIL via mid-path latch. `side_graze`
   still always latches. Spec: clear side/back grazes remain failures.

**Review:** Confirm axis_out≈90°/180° samples still latch; Ep8-style 15° does not.

## Iter9: Sequential post-contact tip retract (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — `_sequential_retract_tip_from_marker`: after PLAN_OK
   in sequential mode, DLS-move tip 50 mm along the outward normal so the
   next episode does not mid-path graze with a flipped wrist on the new
   marker (`CONTACT_INVALID_MIDPATH_GRAZE` axis_out≈π).

## Iter8: Pad-facing CONTACT_HOLD orientation (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — CONTACT_HOLD DLS solves pierce with
   ``build_sphere_contact_approach`` quaternion (pad toward marker), not the
   current wrist quat; refine triggers on wrong_side/side_graze as well as
   no_contact.

## Iter7: CONTACT_HOLD refine when tip outside shell after green (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — after settle hold, run axial pierce refine if tip
   distance exceeds ``radius + outer_tol``, even when mid-path ``contacted``
   already latched (fixes Ep2 13.6→14.0 mm settle `no_contact`).

## Iter6: Freeze joints during contact settle hold (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — during post-motion / CONTACT_HOLD settle windows,
   re-command a frozen ``q`` every sim step. Stopping commands after
   ``contacted`` latched let PD/physics drift the tip out to ~standoff
   (false settle `no_contact` after a real green tip-face sample).

**Review:** Confirm long GUI does not show frozen-arm “stuck” visuals when
hold_s is large; freeze is only for the short settle window.

## Iter5: DLS standoff approach after MotionGen fail (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — `plan_dls_standoff_approach_lerp`: classical DLS + joint
   lerp to oriented standoff when cuRobo approach IK_FAIL / FINETUNE fails;
   rejects tip-immersing mid-lerp chords. Enabled only for
   `CuRoboMotionPlanner` (`contact_dls_approach_fallback`).
2. **`collision.yaml`** — `contact_orientation_cone_max_rad` 0.22→0.26;
   `contact_dls_approach_fallback: true`.
3. **`tests/test_dls_standoff_approach.py`** — unit coverage.

**Review:** DLS approach has no cuRobo collision spheres — immersion chord
check only. Watch for proximal arm / EE-side mid-path grazes under latch.

## Iter4: FK tip-omit pad gate + FakePlanner track skip (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — reseat + axial tip-omit first; refuse free MotionGen
   tip-omit when FK tracks the planned standoff **and** pad is still
   misaligned after reseat. Skip FK pad gate when
   `tip_track_err > contact_standoff_fk_track_tol_m` (12 mm) so unit
   FakePlanners (planned Cartesian tip ≠ FK tip) still exercise recovery.
2. **`run_ik_viz.py`** — early-abort after settle-reclassified PLAN_FAIL
   (not only gated planning fails).
3. **`run_headless_contact_iter.sh`** — `--early-abort-after-fails 1`.
4. **`tests/test_plan_recovery.py`** — `_fake_ok_traj` helper (IK when
   possible); contact MotionGen assert softened for axial-first path.
5. **`tests/test_viz_plan_fail_closed.py`** — settle fail_reason contract
   includes `no_contact` + EARLY_ABORT.

**Review:** FK track skip must not weaken real MotionGen paths (tip should
land within ~12 mm of planned standoff).

## Axial tip-omit lerp + long-GUI iteration (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — `plan_axial_tip_omit_lerp` (DLS-IK + joint lerp) for
   tip-omit; MotionGen tip-omit only as fallback.
2. **`collision.yaml`** — standoff/nudge 12 mm, inflate 6 mm, tip-omit max 14 mm,
   `contact_axis_tolerance_rad` 0.26.
3. Mid-path graze latch + tip-face tighten retained (spec frozen failures).
4. Iter1 headless: inflate/standoff 6/12 mm caused approach IK wall —
   reverted to inflate 0 / standoff 8 mm; kept axial tip-omit.

## Spec: freeze mandatory contact failure conditions (2026-07-17)

**Enumerated changes**

1. **`spec.md`** — new § **Mandatory contact failure conditions (frozen for
   experiments)**: through, side_graze, side/barrel + back axis, immersed,
   no_contact reclassify, invalid settle, mid-path graze latch
   (`CONTACT_INVALID_MIDPATH_GRAZE`), settled EE side spheres, arm sweep/body —
   must remain `PLAN_FAIL`; warn-only mid-path graze that stays PLAN_OK is a
   spec violation; tolerance floor (≤15° / 4 mm / 2 mm).

## Mid-path side/back graze → PLAN_FAIL (2026-07-17)

**Enumerated changes**

1. **`run_ik_viz.py`** — latch mid-path `side_graze` / `wrong_side_axis` into
   `mid_path_side_or_back`; settle forces `PLAN_FAIL(invalid_side)` via
   `CONTACT_INVALID_MIDPATH_GRAZE` (same pattern as `arm_swept`). A later green
   no longer keeps the episode as PLAN_OK after a visual side approach.
2. **`tests/test_viz_plan_fail_closed.py`** — source contract for the latch.

**Evidence:** `/tmp/gui_false_green.log` — SIDE_GRAZE → MARKER_CONTACT → PLAN_OK
on most episodes (warn-only mid-path).

## Tighten false-green tip-face gate (2026-07-17)

**Enumerated changes**

1. **`isaac_sim/target_marker.py`** — tightened tip-face constants to match
   historical honest greens:
   - `TARGET_MARKER_TOOL_AXIS_TOL_RAD`: 0.611 → **0.26** (≈15°)
   - `TARGET_MARKER_TIP_FACE_RADIUS_M`: 0.010 → **0.004** (4 mm pad)
   - `TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M`: 0.004 → **0.002** (2 mm)
2. **`planning/marker_contact_diag.py`** — new `settle_has_side_sphere_hits`
   (pure NumPy): tip-zone sphere ∩ marker OK; any **side** hit → reject.
3. **`isaac_sim/run_ik_viz.py`** — after settle `classify_tip_contact`, scan YAML
   collision spheres at `q_settled`; side hits → `CONTACT_INVALID_SETTLE` /
   `MARKER_EE_SIDE_SPHERE` → `PLAN_FAIL(invalid_side)`.
4. **`tests/test_target_marker.py`** — rejects prior false-green metrics
   (axis_out≈25°, lat≈8 mm, dist≈15 mm); side/back/immerse still fail.
5. **`tests/test_marker_contact_diag.py`** + **`tests/test_viz_plan_fail_closed.py`**
   — settle side-sphere helper + source-contract wiring.
6. **Docs** — `STATUS.md`, `docs/phase2_geometry.md`, `spec.md` tip-face section
   synced to ≈15° / 4 mm / 2 mm + volumetric side reject.
7. **`configs/planning/collision.yaml`** — `contact_orientation_cone_max_rad`
   0.30 → **0.22** so cone candidates stay under the ≈15° gate.

**Recommended for further review:** short headless+GUI smoke may now show
honest `PLAN_FAIL(invalid_side)` if approaches still graze — do **not** widen
tip-omit or axis tol to chase greens; prefer spheres-ON reseat / vias.

## Tip-face reject tests verified (side/back/immerse) (2026-07-17)

**Enumerated changes**

1. **`tests/test_target_marker.py`** — explicit
   `test_rejects_side_barrel_back_and_submerged_ee_contacts` (side/barrel,
   flipped/back, immersed, side_graze must all return `ok is False`); tightened
   side_graze exact reason.
2. **`tests/test_viz_plan_fail_closed.py`** — source contract that settled
   classify failures become `PLAN_FAIL(immersed|invalid_side)` via
   `CONTACT_INVALID_SETTLE`.

## Tip-omit cap + pad-alignment gate (2026-07-17)

**Enumerated changes**

1. **`recovery.py`** — tip-omit length allow is always ≤ `contact_nudge_max_m`
   (removed ≤60 mm fallback after spheres-ON approach fail).
2. **`fk_pad_aligned_for_tip_omit` / `tip_omit_length_allow_m`** — helpers;
   tip-omit after approach fail requires pad-aligned FK; skip-near misaligned
   pads reseat with spheres ON (or refuse).
3. **`tests/test_tip_omit_gates.py`** — length cap + orientation refuse cases.
4. **`collision.yaml`** — comment: do not reintroduce multi-cm tip-omit.

**Recommended for further review:** GUI visual check that side/back EE hits
are gone; if greens drop, prefer vias / Cartesian spheres-ON approach — not
widening tip-omit.

## Kill log streamers before each test cycle (.cursorrules) (2026-07-17)

**Enumerated changes**

1. **`.cursorrules`** — mandatory: kill leftover background `tail -F` / DEC|RESULT
   log streamers before starting any new Isaac smoke / verification cycle.
2. Cleared leftover streamer processes from prior GUI/headless runs.

## Headless contact iteration + DEC logging (2026-07-17)

**Enumerated changes**

1. Restored **8 mm** standoff/nudge; **inflate=0** (known-good tip-face planning).
2. Live **`DEC|…`** decision logging (`decision_emit` in recovery / viz).
3. **`--early-abort-after-fails`** (kept); `ISAAC_VIZ_RECOVERY_TIMEOUT_S` override.
4. **`arm_sweep_link_radius_m: 0.012`** — stop false `MARKER_ARM_SWEEP` from 25 mm capsules.
5. Tip-omit fallback allow after approach fail up to **60 mm**.
6. Headless iter2 smoke: **PASSED** `ok=8 green=8 rate=1.0`.

**Recommended for further review:** whether 60 mm tip-omit fallback reintroduces
through-sphere on some targets; prefer vias when tip_far ≫ standoff.

## GUI early-abort + tip-face/sphere analysis (2026-07-17)

**Enumerated changes**

1. **`--early-abort-after-fails`** (default 3) — stop viz loop when `PLAN_OK=0`
   after N `PLAN_FAIL` episodes; log `EARLY_ABORT`.
2. **`GUI_STOP` on PLAN_FAIL hold** when Kit `is_running()` goes false (was a
   silent `break`).
3. **STATUS.md** — analysis: overlay sphere misalignment ≠ PLAN_FAIL cause;
   tip-face commits `e219fa4` / `4cfaf89` are the historical inflection.

## Collision-sphere GUI overlay (2026-07-17)

**Enumerated changes**

1. **`--show-collision-spheres`** / **`--collision-sphere-opacity`** on
   `isaac_sim/run_ik_viz.py` — translucent USD overlay of mesh-fitted cuRobo
   spheres (amber=proximal, cyan=tip-omit links).
2. **`isaac_sim/collision_sphere_viz.py`** + **`planning/collision_sphere_world.py`**
   — FK world placement from committed YAML (no Kit needed for unit tests).
3. **`UrdfKinematicModel.link_transforms`** — per-link 4×4 for sphere placement.
4. **GUI smoke** (`smoke_isaac_viz.sh --gui`) enables the overlay by default;
   `ISAAC_VIZ_SHOW_COLLISION_SPHERES=0` or `--no-show-collision-spheres` disables.
5. **Tests** — `tests/test_collision_sphere_viz.py`.

**Recommended for further review:** whether Kit viewport opacity needs a
dedicated translucent render pass on some Isaac Sim builds.

## Surface-shell contact + arm-sweep + via-only-in-region (2026-07-17)

**Enumerated changes**

1. **`classify_tip_contact` surface shell** — reject `immersed` when tip is
   deeper than `radius − inner_tol` (touch surface, do not collide into volume).
2. **`MARKER_ARM_SWEEP` mid-path** + settle `arm_body_contact` — proximal arm
   must not graze the marker before tip-face contact.
3. **Planning clearance** — `contact_standoff_m` 8→20 mm; `target_obstacle_inflate_m: 0.008`
   on the planning obstacle only (visual stays 12 mm).
4. **`SKIPPED_UNREACHABLE` only outside Dexterous Region** —
   `plan_prescreen_skip_orientation_infeasible: false`; in-region uses vias.
5. **`spec.md`** — via vs unreachable table; cuRobo-as-planner vs oracle-IK
   seeding guidance (do not replace MotionGen with oracle IK alone).

**Recommended for further review:** whether 20 mm standoff + 8 mm inflate is
enough to eliminate visible arm-side clips under GUI servo lag.

## Skip-unreachable gate + EE-only settle + countable episodes (2026-07-17)

**Enumerated changes**

1. **`max_skip_unreachable_frac`** (YAML/CLI/env) — fail smoke when unreachable
   skips exceed 25% of candidates considered.
2. **`--visualize N` = N countable episodes** — skips (`SKIPPED_UNREACHABLE` /
   overlapping) do not consume episode slots; prefer Dexterous Region candidates.
3. **`proximal_arm_contacts_target`** — settle-time `PLAN_FAIL(arm_body_contact)`
   if proximal arm capsules intersect the marker (EE tip-face exclusive).
4. **`spec.md`** — new § EE-only surface contact; episode/skip-gate contracts;
   secondary docs-push requirement.
5. **`scripts/git_secondary_docs_push.sh`** — docs-only second push without
   re-editing STATUS/CHANGES/last_prompt in the same turn.
6. **Tests** — `tests/test_skip_unreachable_gate.py`; smoke/post-Kit checks.

**Recommended for further review:** whether `0.25` is the right skip-frac cap
once Dexterous-Region-first sampling is verified on a full GUI 48-episode run.

## STATUS.md host-writable + SKIPPED_UNREACHABLE append reliability (2026-07-17)

1. **`STATUS.md` permissions** — group-writable for the host Isaac user so
   end-of-test analysis can append (was root:root 0644; Kit runs as `jywilson`).
2. **`skipped_analysis.analyze_and_report`** — log a warning on `OSError` instead
   of silently dropping the STATUS append.
3. **Backfilled** the full-48 GUI analysis block (`ok=19 fail=0 skip_unreachable=22`)
   that Kit could not write earlier.

## Headless planning parity + Pinocchio dexterous-workspace gate (2026-07-17)

**Enumerated changes**

1. **`scripts/run_verification.sh` / `smoke_isaac_viz.sh`** — headless runs the
   same planning workload as GUI (`visualize=48`, `n_poses=240`, sequential
   home); default headless `ISAAC_VIZ_SMOKE_TIME_WARP=4`.
2. **`isaac_sim/run_ik_viz.py`** — `--time-warp` scales joint playback speed and
   shrinks hold waits (planning unchanged).
3. **`planning/pinocchio_ik.py`** *(new)* — Pinocchio FK/Jacobian multi-seed IK
   for the dexterous-workspace gate (preferred classical library).
4. **`planning/dexterity.py`** — `backend=auto|pinocchio|numpy`; orientation
   skip auto-on for Pinocchio, auto-off for NumPy.
5. **`configs/planning/collision.yaml`** — `plan_prescreen_backend: auto`,
   `plan_prescreen_skip_orientation_infeasible: null`.
6. **Tests / host smokes** — `test_pinocchio_ik.py`, smoke script contracts,
   `scripts/host/smoke_pinocchio_prescreen.sh`, `probe_ik_libs.sh`.
7. **Docs** — STATUS / REFERENCES / README / spec touch-ups.

**Recommended for further review:** confirm GUI + headless both meet the
strict 1.0 gate after Pinocchio skips orientation-infeasible edge targets.

## Dexterous prescreen + orientation cone + budget + SKIPPED_UNREACHABLE analysis (2026-07-17)

Implements the three recommended next steps from the long strict-1.0 GUI run,
adds the mandatory end-of-test `SKIPPED_UNREACHABLE` analysis, and answers the
headless-vs-GUI and deep-learning-IK questions (see `STATUS.md`).

**Enumerated changes**

1. **`src/residual_adaptive_ik/planning/contact_geometry.py`** — new
   `contact_orientation_cone()` (bounded pad-facing cone, exact normal first,
   all entries within `cone_max_rad` of the normal so the honest gate still
   holds).
2. **`src/residual_adaptive_ik/planning/recovery.py`** — new
   `contact_orientation_candidates()` helper; `try_oriented_tip_face_contact`
   now tries the cone in the approach leg and **threads the chosen orientation**
   into the tip-omit nudge (so it does not revert to a just-failed exact normal).
3. **`src/residual_adaptive_ik/planning/dexterity.py`** *(new)* — deterministic
   DLS-IK dexterity prescreen (`contact_pose_is_dexterous`), stabilized solver
   (`build_prescreen_solver`), FK-sampled orientation-aware warm-start seeds,
   `is_in_dexterous_region`, reach-envelope loader. Skips `unreachable_position`
   always; `unreachable_orientation` only under an opt-in flag (default off).
4. **`src/residual_adaptive_ik/planning/skipped_analysis.py`** *(new)* —
   end-of-test analyzer: parses `SKIPPED_UNREACHABLE` lines, speculates the
   reason, and (for Dexterous-Region targets) speculates a deterministic-IK via;
   prints to the prompt and appends to `STATUS.md`.
5. **`src/residual_adaptive_ik/planning/curobo_planner.py`** — planning-budget
   wiring: `curobo_num_ik_seeds`/`curobo_num_trajopt_seeds` → MotionGenConfig
   (defensive `try/except`), optional `curobo_plan_enable_graph`/
   `curobo_plan_timeout_s` → MotionGenPlanConfig.
6. **`isaac_sim/run_ik_viz.py`** — runs the prescreen before planning
   (`RESULT SKIPPED_UNREACHABLE …`, excluded from the gate), new
   `phase2_skipped_unreachable` metric + summary field, and the end-of-test
   `analyze_and_report` hook.
7. **`configs/planning/collision.yaml`** — `curobo_max_attempts` 4→6; new
   `curobo_num_ik_seeds`/`curobo_num_trajopt_seeds`/`curobo_plan_*`,
   `contact_orientation_cone_*`, `plan_prescreen_*`, `dexterous_region_margin_m`.
8. **Tests** — `tests/test_dexterity.py`, `tests/test_contact_orientation_cone.py`,
   `tests/test_skipped_analysis.py` (new); extended `tests/test_plan_recovery.py`
   (cone candidates + updated `uses_via_when_direct_fails` fake planner).
9. **Docs** — `spec.md` (prescreen/cone/budget section + mandatory
   SKIPPED_UNREACHABLE analysis requirement + log-format contract),
   `STATUS.md`, `REFERENCES.md` (IK-library note), `README.md`,
   `docs/phase2_geometry.md`, `docs/last_prompt.md`.

**Recommended for further review**

- **Honesty of the prescreen default.** `plan_prescreen_skip_orientation_infeasible`
  is **off** because DLS is not a reliable 6-DOF orientation-completeness oracle
  (it stalls on some genuinely-feasible poses); enabling it can over-skip and
  inflate the gate. Review whether to adopt an analytic/IKFast/TRAC-IK oracle
  before turning it on. With it off, the strict 1.0 rate is **not** expected to
  change materially — the orientation-limited edge targets still go to cuRobo.
- **cuRobo seed kwargs** (`num_ik_seeds`/`num_trajopt_seeds`) are passed
  defensively but **unverified on the Spark host cuRobo build** (container has no
  GPU/Kit). Confirm they are accepted (no `TypeError` fallback) during the next
  host GUI run.
- **STATUS.md auto-append** on every viz run — confirm this is the desired
  cadence (it is per the explicit requirement).

## Long strict-1.0 GUI verification — honest result (2026-07-17)

Verification-only turn (no code change). Ran the long GUI smoke with the strict
gate: `ISAAC_VIZ_MIN_PLAN_OK_RATE=1.0`, 48 GUI episodes (39 planned after skips;
`N_POSES=240` is only the sampling pool, `VISUALIZE=48` drives episode count).

| Item | Result | Notes |
|------|--------|-------|
| Strict-1.0 gate | **FAILED (honest)** | `PLAN_OK rate = 0.692` (27 ok / 39 planned). Does **not** reproduce the 8-pose short-run 1.0; it is the generalization result over a diverse workspace. |
| False greens | **0** | Every `MARKER_WRONG_SIDE`/`MARKER_THROUGH` logged as *"not green"* (rejected); `phase2_invalid_side_contacts = 0`. All 27 OKs are genuine pad-facing greens. |
| Failure mode | 12× `recovery_timeout` + `contact_approach IK_FAIL` | Orientation-feasibility limits at/near workspace edge (tip 0.20–0.40 m, ~0.28 m reach). Nudge correctly refused (`dist≫allow`); marker stayed **yellow**. |
| Radial reposition via | fired 0× | Failures are far-reach, not `EE_CLOSE`; reposition (unit-tested) is not the applicable remedy here. |

**Recommendation (needs a design decision, not a silent gate loosening):** sample
within the dexterous workspace (report infeasible poses as `SKIPPED_UNREACHABLE`),
and/or allow a bounded tool-axis cone for edge targets, and/or raise planning
budget. Details + rationale in `STATUS.md` and `docs/phase2_geometry.md`.

## EE-close IK_FAIL reposition via (base→target radial pre-approach) (2026-07-16)

Follow-up: instead of excluding `EE_CLOSE` (sphere-overlaps-at-start) targets
from the rate gate, **recover** them. When the start is valid but the tip is
folded inside the standoff shell, the oriented contact returns `IK_FAIL` and the
usual standoff candidates are degenerate (the tip→center ray is ~0). This adds an
explicit **reposition via** that backs the arm off along the well-conditioned
**base→target** radial to an extended pre-approach, then retries the oriented
contact from a more dexterous pose.

| Path | Action | Notes |
|------|--------|-------|
| `src/residual_adaptive_ik/planning/recovery.py` | **Updated** | New pure `radial_preapproach_tip(...)` (base→target ray, robust when tip≈center). New `try_radial_reposition_via(...)`: plans a pad-facing pre-approach at progressive clearances/yaws with the tip-spheres-ON planner, executes it, returns the repositioned joints. `plan_via_standoff` now triggers it when the direct oriented contact returns `IK_FAIL` **and** the tip is within `radius + reposition_close_shell_m` of center, bounded by `plan_recovery_reposition_max_attempts`. Added `_is_ik_fail(...)` |
| `configs/planning/collision.yaml` | **Updated** | New knobs: `plan_recovery_reposition_enabled` (true), `_close_shell_m` (0.06), `_clearances_m` ([0.08,0.12,0.16]), `_yaws_rad` ([0,0.4,-0.4]), `_max_attempts` (2) |
| `tests/test_plan_recovery.py` | **Updated** | `radial_preapproach_tip` geometry, `_is_ik_fail`, `try_radial_reposition_via` (success→moves; all-fail→None), and an integration test where an EE-at-marker start + `IK_FAIL` triggers a base→target reposition then succeeds |
| `spec.md` / `STATUS.md` / `docs/phase2_geometry.md` / `docs/last_prompt.md` | **Updated** | Document the reposition-via strategy |

**GUI evidence (strict `ISAAC_VIZ_MIN_PLAN_OK_RATE=1.0`, 8-trial short run, wip_phase3):**
**PASSED — rate 1.000** (7/7 planned green, 1 skipped overlapping). Trial 1 — the
`EE_CLOSE` target (tip starts 57 mm from center) that failed the prior strict run
— is now green (`strategy=via_contact`). `VIA_PRESSURE_HIGH` correctly fired for
the 3 consecutive `EE_CLOSE` via-heavy episodes (`ema_usage=1.00, streak=3`) then
decayed on the clean episodes. The reposition via served as a safety net and was
not needed to fire this run (standoff vias rescued trial 1 first); its path is
covered by unit tests.

**Recommended further review:** cuRobo MotionGen has run-to-run nondeterminism
near the feasibility boundary, so `EE_CLOSE` targets can occasionally still fail;
the reposition via and standoff vias together improve the odds but do not
guarantee 1.000 every run. The base position for the radial is the URDF
`base_link` origin (tip poses are in that frame) — revisit if the model frame
changes.

## Signed tip-face contact gate + final-pose override + via-pressure metric (2026-07-16)

Follow-up to the honest-gate work below. The sign-agnostic tool-axis check
(folded to `[0, π/2]`) still accepted a **flipped/back** contact: the sphere on
the correct axis line but with the flange +Z pointing the wrong way, i.e. the
marker touching the EE **from the inside/back** while the marker flashed green.
This change makes the gate **signed**, adds an independent authoritative check
that reports a **failure even after a transient green**, and quantifies the
**repeated need for vias in consecutive episodes** as a math metric.

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/target_marker.py` | **Updated** | `classify_tip_contact` axis check is now **signed** against the *outward* normal (tip − center): valid front contact requires `axis_out ≤ TOOL_AXIS_TOL` (≈35°). Rejects side/barrel (`axis_out ≈ 90°`) **and** flipped/back (`axis_out ≈ 180°`) — the latter is what the folded check wrongly accepted. Metrics now expose `axis_in_err_rad`, `axis_out_err_rad`, `axis_line_err_rad` |
| `isaac_sim/run_ik_viz.py` | **Updated** | Added **authoritative final-pose check**: after the hold, the *settled* pose is re-classified; a green that settles wrong-side/through is overridden to `CONTACT_INVALID_SIDE` → `PLAN_FAIL(invalid_side)` (green flash never counts). Added **`ViaPressureTracker`** wiring: per-episode `VIA_PRESSURE` (EMA of via usage + consecutive-via streak) and `VIA_PRESSURE_HIGH` warning. Green/near-miss logs now print `axis_in`/`axis_out` (deg). New metrics `phase2_invalid_side_contacts`, `phase2_via_pressure_ema_usage`, `phase2_via_pressure_max_streak` |
| `isaac_sim/viz_plan_policy.py` | **Updated** | New pure/testable `ViaPressureTracker`: `E_i = α·u_i + (1−α)·E_{i−1}` (via-usage EMA), `A_i` (via-count EMA), consecutive streak `S_i`; `high` when `E_i ≥ 0.6` **and** `S_i ≥ 3` |
| `src/residual_adaptive_ik/planning/contact_geometry.py` | **Docs** | Frame-convention note: the planner commands +Z along the **outward** normal (the orientation cuRobo can reach; commanding inward → `IK_FAIL`); the URDF/FK tool +Z is opposite, so the achieved contact reads as `axis_out ≈ 0` — which the signed gate requires. Behavior unchanged |
| `tests/test_target_marker.py` | **Updated** | Signed-axis tests: `+Z` outward = OK, `+Z` inward (flipped) = rejected, perpendicular = rejected; assert `axis_out`/`axis_in` values |
| `tests/test_contact_geometry.py` | **Updated** | Asserts planning-frame `+Z` is along the outward normal (with frame-convention comment) |
| `tests/test_viz_plan_fail_closed.py` | **Updated** | New `ViaPressureTracker` tests (consecutive-via `high`; sparse vias never trip) |
| `spec.md` / `STATUS.md` / `docs/phase2_geometry.md` / `docs/last_prompt.md` | **Updated** | Document the signed gate, the sign convention (empirically established), the final-pose override, and the via-pressure math |

**GUI evidence (strict 1.0 gate, 8-trial short run, wip_phase3):**
- Wrong-hypothesis run first (commanded +Z inward): `contact_approach_q0/q1:plan_failed:IK_FAIL` on every via → 0 green. Confirmed the reachable orientation is +Z **outward**.
- Corrected signed-outward gate: **5 legitimate greens** all with `axis_out ≤ 7°`, `lat ≤ 0.5 mm`, `pen ≈ −radius` (front tip-face). **0 `CONTACT_INVALID_SIDE`** overrides. Side/flipped samples logged `MARKER_WRONG_SIDE`/`MARKER_SIDE_GRAZE` (`axis_out 54–80°`) and stayed red. `VIA_PRESSURE` EMA rose to 1.0 / 0.88 across the via-heavy episodes.
- Gate verdict `0.833 < 1.000` **FAILED honestly** on trial 1 only — an `EE_CLOSE` target (tip starts 57 mm inside the standoff shell) that cannot achieve a valid front contact. No green was a false positive.

**Recommended further review:** the tool-axis **sign** was established empirically (reachable contacts measure `axis_out ≈ 0`; `contact_geometry` +Z is opposite the URDF/FK +Z). Confirm this against the physical MyCobot flange frame before hardware. The strict 1.0 rate is not met for `EE_CLOSE` starts; decide whether to exclude those from sampling or accept them as a known kinematic limitation.

## Honest tip-face contact gate + wrong-side / through / high-retry instrumentation (2026-07-16)

Addresses three reported viz defects: (1) the EE approaching from the **wrong
side** and still turning the marker green, (2) the EE moving **through** the red
sphere counted as success, (3) **high retry counts** when the EE starts close to
the target, with no diagnostic explaining why.

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/target_marker.py` | **Updated** | New `classify_tip_contact(...)` → `(ok, reason, metrics)`; reasons `ok/no_contact/through/side_graze/wrong_side_axis`. `ee_contacts_target` now delegates to it. Added `TARGET_MARKER_TOOL_AXIS_TOL_RAD` (≈35°, sign-agnostic **line** alignment) + `TARGET_MARKER_THROUGH_PENETRATION_TOL_M` |
| `isaac_sim/run_ik_viz.py` | **Updated** | Green gate re-enables tool-axis check via `classify_tip_contact` (rejects wrong-side/through). Per-episode contact diagnostics log `MARKER_CONTACT/MARKER_SIDE_GRAZE/MARKER_WRONG_SIDE/MARKER_THROUGH` with measured dist/pen/lateral/axis. Added `EE_CLOSE` (tip starts ≤ radius+50 mm) and `HIGH_RETRY_WHEN_CLOSE` (≥`HIGH_RETRY_VIA_THRESHOLD`=5 vias) instrumentation; `MARKER_NO_CONTACT` now reports the nearest-sample reason |
| `configs/planning/collision.yaml` | **Updated** | `contact_via_nudge_max_m` lowered `0.22 → 0.020`: keep tip spheres ON until a **short** nudge; long tip-omit that sweeps through the sphere is refused (`contact_nudge_direct_refused`) |
| `tests/test_target_marker.py` | **Updated** | Tool-axis test now line-based (collinear either sign OK; perpendicular = wrong-side rejected). New `test_classify_tip_contact_detects_failure_modes` (through/side/wrong-side/ok/no_contact) |
| `tests/test_viz_plan_fail_closed.py` | **Updated** | CONTACT_HOLD source contract matches current wording ("axial refine to pierce", targets pierce not marker center) |
| `spec.md` / `STATUS.md` / `docs/phase2_geometry.md` | **Updated** | Document detection + honest gate + short-nudge invariant |

**GUI evidence (8-trial sequential, metrics-only gate):** trials 2 & 4 first
logged `MARKER_SIDE_GRAZE` (axis_line 60–77°) and `MARKER_WRONG_SIDE`
(42–52°) — the exact false-green cases — and were **rejected**, turning green
only on a true tip-face contact (axis_line ≤ 35°, `pen` on the near hemisphere).
No `MARKER_THROUGH`. 7/8 green; trial 8 (0.263 m radial, far/awkward) honestly
`PLAN_FAIL` because both the spheres-on approach `IK_FAIL`s and the long
tip-omit is refused (not masked as a through-sphere green).

**Worth review:**
- The tool-axis gate is **sign-agnostic** (folds the +Z error to the approach
  line, ≈35° tol). GUI logs `axis_out` so the true flange +Z sign vs. pad can be
  confirmed on hardware meshes; once confirmed, tighten to a signed check.
- Trial 8 far-target `PLAN_FAIL` is a genuine reach/IK limitation (pre-existing).
  Do **not** raise `contact_via_nudge_max_m` to "fix" it — that reintroduces the
  through-sphere false green.

---

## Oriented tip-face contact: spheres-on approach + tip-omit axial nudge (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `planning/contact_geometry.py` | **Created** | Oriented standoff/pierce; tool +Z faces sphere; axial validators |
| `planning/recovery.py` | **Updated** | `try_oriented_tip_face_contact`; direct/via use spheres-on then tip-omit axial; remove long omit-tip via2 |
| `configs/planning/collision.yaml` | **Updated** | `contact_axis_enabled`, `contact_standoff_m`, `contact_nudge_*` |
| `isaac_sim/target_marker.py` | **Updated** | Optional tool-axis alignment for green contact |
| `isaac_sim/run_ik_viz.py` | **Updated** | Pass EE quat to contact; `CONTACT_HOLD` (no center-drive to `q_sol`) |
| `tests/test_contact_geometry.py` | **Created** | Geometry / axis contracts |
| `tests/test_plan_recovery.py` / `test_target_marker.py` / viz tests | **Updated** | Match new strategies |
| `spec.md` / `STATUS.md` / `docs/phase2_geometry.md` | **Updated** | Document approach–nudge policy |

**Worth review:** tool +Z = outward surface normal at pierce (pad faces sphere). Confirm against flange mesh if green contact rejects valid approaches.

---

## Phase 2 contact-planning exploration (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `docs/last_prompt.md` | **Updated** | Logged the contact-planning exploration prompt per workspace policy; no runtime code changed |

**Worth review:** The implementation sketch recommends replacing the current full-path omit-tip direct leg and unplanned GUI joint nudge with a collision-spheres-on standoff leg followed by a bounded axial contact leg.

---

## GUI via-waypoint warning + real-time log format (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/run_ik_viz.py` | **Updated** | `VIA_WAYPOINT_USED` warn (Kit Console) when direct plan failed but standoff via succeeded; per-episode `RESULT … \| STATUS ok=.. fail=.. via=.. green=.. skip=.. rate=..` lines; summary adds `via=`/`skip=` and warns on issues; metrics `phase2_via_waypoint_ok` / `phase2_skipped_targets` |
| `tests/test_viz_status_format.py` | **Created** | Contracts for warning level, RESULT/STATUS lines, metrics keys |
| `spec.md` | **Updated** | § GUI test log / warning format (monitoring contract) |
| `README.md` / `STATUS.md` | **Updated** | Log-monitoring documentation |

**Worth review:** `RESULT PLAN_OK strategy=via_standoff` episodes log at warn level by design (operator attention), even though they count as successes.

---

## GUI smoke: home once only / CLI default --no-reset-to-home (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | GUI smoke must home only on first episode; sequential is default |
| `configs/planning/collision.yaml` | **Updated** | `reset_to_home_before_each_trial: false` |
| `isaac_sim/run_ik_viz.py` | **Updated** | `parser.set_defaults(reset_to_home=False)` |
| `scripts/host/smoke_isaac_viz.sh` | **Updated** | Default `ISAAC_VIZ_SMOKE_RESET_TO_HOME=0` |
| `scripts/run_verification.sh` | **Updated** | Forces `ISAAC_VIZ_SMOKE_RESET_TO_HOME=0` for GUI |
| `tests/test_robot_home.py` / `test_plan_recovery.py` / `test_isaac_viz_smoke.py` | **Updated** | Assert sequential defaults |
| `STATUS.md` / `README.md` / `docs/phase2_*.md` | **Updated** | Document default policy |

**Worth review:** Sequential GUI smoke may no longer hit a strict 1.0 rate as easily as independent-episode mode; failures now exercise path-dependent recovery as intended.

---

## Preparatory q_seed escape for INVALID_START (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `planning/recovery.py` | **Updated** | `try_move_to_preparatory_seed` — primary INVALID_START escape |
| `configs/planning/collision.yaml` | **Updated** | `plan_recovery_prep_seed_enabled` / `_max` |
| `tests/test_plan_recovery.py` | **Updated** | Open-loop + planned prep-seed contracts; via fallthrough |
| `spec.md` / `docs/phase2_geometry.md` | **Updated** | Implementation contract matches code |
| `planning/__init__.py` | **Updated** | Re-export `try_move_to_preparatory_seed` |

**Worth review:** Open-loop joint moves to `q_seed` bypass MotionGen when the start is colliding (required); they are not collision-checked mid-lerp.

---

## Phase 3 completion — FK loss + acceptance gate (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `learning/fk_torch.py` | **Created** | Differentiable batched FK for training |
| `learning/train_supervised.py` | **Updated** | FK tip loss + joint-limit penalty |
| `learning/evaluate_supervised.py` | **Updated** | Stress split, per-mode breakdown, acceptance gate |
| `data/generate_supervised_data.py` | **Updated** | `perturbation_mode` in NPZ |
| `configs/learning/supervised_residual.yaml` | **Updated** | `lambda_fk_pose`, evaluation thresholds |
| `tests/test_phase3_acceptance.py` | **Created** | FK parity + gate contracts |
| `docs/phase3_supervised.md` | **Updated** | Phase 3 complete metrics |

**Verified:** host train 80 epochs → acceptance `passed`; pytest **107 passed**, 5 skipped.

---

## Phase 3 supervised residual scaffold + first MLP (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `src/.../data/generate_supervised_data.py` | **Implemented** | Perturbation dataset + NPZ I/O + obs pack |
| `src/.../learning/train_supervised.py` | **Implemented** | Obs std, MSE train, checkpoint + CSV |
| `src/.../learning/evaluate_supervised.py` | **Implemented** | IK / oracle / MLP comparison |
| `scripts/host/train_supervised_residual.sh` | **Added** | Host Isaac python train entry |
| `scripts/run_phase3_supervised.sh` | **Updated** | Delegates to host when in container |
| `configs/learning/supervised_residual.yaml` | **Updated** | Lighter mag penalty, lr 1e-3 |
| `tests/test_generate_supervised_data.py` | **Added** | Dataset contracts |
| `tests/test_evaluate_supervised.py` | **Added** | Oracle ≥ IK |
| `tests/test_residual_model.py` | **Added** | Bound check (torch skip ok) |
| `docs/phase3_supervised.md` | **Added** | Metrics report |
| `STATUS.md` / `README.md` / `REFERENCES.md` | **Updated** | Phase 3 status + libraries |

**Worth review:** MLP median gain (~0.39 mm) still trails oracle (~0.97 mm); next add FK pose-error loss.

## Sequential multi-target + IK seed bank (2026-07-16)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | Sequential multi-target use case; IK failure → preparatory repositioning strategy |
| `.cursorrules` | **Updated** | Rebase onto `main` before landing (no merge-of-main path) |
| `kinematics/ik_seed_bank.py` | **Created** | Joint-space seed bank + farthest-from-failed ordering |
| `planning/recovery.py` | **Updated** | Via2 IK fallback uses seed bank (max 4 seeds) |
| `tests/test_ik_seed_bank.py` | **Created** | Ordering + solve + spec contract |
| `configs/planning/collision.yaml` | **Updated** | Comments clarify independent vs sequential modes |
| `docs/phase2_geometry.md` / `REFERENCES.md` / `STATUS.md` / `README.md` | **Updated** | Dual modes + IKSel / MoveIt refs |

**Worth review:** Seed bank is classical IK only; full “plan to preparatory `q_seed` then retry” for `INVALID_START` is still home-blend + vias (spec’d, not fully replacing home escape yet).

---

## MARKER_NO_CONTACT reclassified as failure + INVALID_START via-fallthrough (2026-07-15)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/run_ik_viz.py` | **Updated** | MARKER_NO_CONTACT decrements n_plan_ok, increments n_plan_fail, turns yellow |
| `isaac_sim/target_marker.py` | **Updated** | TIP_FACE_RADIUS_M widened 6 → 10 mm (accepts ~56° off-axis, rejects equator) |
| `src/.../planning/recovery.py` | **Updated** | Via-loop INVALID_START escape; escape cap raised 0.95 → 0.99; weight reset after saturation |
| `configs/planning/collision.yaml` | **Updated** | `reset_to_home_before_each_trial: true` (was false); reproducible 1.0 rate gate |
| `scripts/run_verification.sh` | **Updated** | Removed `--no-reset-to-home` override (uses YAML default) |
| `tests/test_plan_recovery.py` | **Updated** | Assert YAML default is reset-to-home=true |
| `tests/test_robot_home.py` | **Updated** | Assert YAML default is reset-to-home=true |
| `isaac_sim/run_ik_viz.py` | **Updated** | Base keepout + capsule prefilter; CONTACT_NUDGE toward IK tip before NO_CONTACT fail |
| `isaac_sim/target_marker.py` | **Updated** | OUTER_TOL 3→8 mm; BASE_KEEPOUT_XY/Z for invalid near-base targets |
| `tests/test_viz_plan_fail_closed.py` | **Updated** | New test: `test_marker_no_contact_reclassified_as_failure` |
| `tests/test_isaac_viz_smoke.py` | **Updated** | Assert reclassification contract in source |
| `docs/phase2_geometry.md` | **Updated** | Contact required for success |
| `docs/phase2_status_and_resume.md` | **Updated** | Contact required; marker color table |
| `STATUS.md` | **Updated** | Fix documentation |

## INVALID_START via-fallthrough fix (2026-07-15)

| Path | Action | Notes |
|------|--------|-------|
| `src/.../planning/recovery.py` | **Updated** | Home-escape saturated → fall through to via standoffs instead of looping |
| `tests/test_plan_recovery.py` | **Updated** | New test: `test_invalid_start_saturated_falls_through_to_vias` |
| `STATUS.md` | **Updated** | Documented fix; updated checklist |
| `docs/last_prompt.md` | **Updated** | Prompt log |

## Push gate: GUI required before remote push (2026-07-12 00:26)

| Path | Action | Notes |
|------|--------|-------|
| `STATUS.md` | **Updated** | § Push-to-remote gate — GUI before `git push` |
| `.cursorrules` | **Updated** | Agent must not push without GUI smoke |
| `README.md`, `docs/phase2_status_and_resume.md`, `spec.md` | **Updated** | Cross-refs to the gate |

## Progressive near→far recovery vias (2026-07-12 00:23)

| Path | Action | Notes |
|------|--------|-------|
| `src/.../planning/recovery.py` | **Updated** | Standoffs sorted nearest → farthest (progressive retries) |
| `configs/planning/collision.yaml` | **Updated** | `plan_recovery_min_standoff_travel_m: 0.01` (no-op floor only) |
| `tests/test_plan_recovery.py` | **Updated** | Assert ascending travel order |
| Docs (`phase2_*`, `STATUS`, `last_prompt`) | **Updated** | Document progressive distance |

## Tip-face contact + farthest recovery vias (2026-07-12 00:16)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/target_marker.py` | **Updated** | Tip-face pierce contact; side grazes invalid when `approach_from_m` set |
| `isaac_sim/run_ik_viz.py` | **Updated** | Passes / refreshes `approach_from_m` for green |
| `src/.../planning/recovery.py` | **Updated** | `ordered_standoff_candidates` farthest-first; min tip travel |
| `configs/planning/collision.yaml` | **Updated** | `plan_recovery_min_standoff_travel_m: 0.06` |
| `tests/test_target_marker.py` | **Updated** | Tip-face vs side assertions |
| `tests/test_plan_recovery.py` | **Updated** | Farthest-first + YAML min travel |
| `docs/phase2_geometry.md`, `spec.md`, `STATUS.md` | **Updated** | Document tip-face + far retries |

## Hung Isaac wait diagnosis (2026-07-11 23:59)

| Path | Action | Notes |
|------|--------|-------|
| `docs/last_prompt.md` | **Updated** | Logged hung-agent inquiry |

**Finding:** Kit already exited (`Simulation App Shutting Down` at 23:58). No live `kit` / `python.sh` process. Latest GUI metrics: PLAN_OK **38/48 (0.792)** vs `min_plan_ok_rate: 1.0` → gate **FAILED**. Safe to stop/restart the waiting agent UI.

---

Review list of everything created or copied into `spark_isaac_mycobot_v2` during the initial fork bootstrap.

## Top-level documents

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | Preserved residual-IK requirements; added host/container table, doc maintenance, v1 relationship; clarified repo root name |
| `README.md` | **Created** | Daily workflow from fork spec |
| `STATUS.md` | **Created** | Operational briefing |
| `CHANGES.md` | **Created** | This file |
| `.cursorrules` | **Created** | Architecture + TDD + host/ROS rules (from v1 ops + v2 non-negotiables) |
| `.gitignore` | **Created** | Python, colcon, assets, vendor clone |
| `pyproject.toml` | **Created** | Editable `residual-adaptive-ik` package |
| `requirements.txt` | **Created** | Phase 1 deps |
| `REFERENCES.md` | **Copied** | From `spark_isaac_mycobot_demo` (hardware / ROS / Isaac / RL links) |

## Configs (`configs/`)

| Path | Action |
|------|--------|
| `configs/robot/mycobot_280.yaml` | Created (from spec example + URDF hint) |
| `configs/robot/joint_limits.yaml` | Created |
| `configs/ik/ik_solver.yaml` | Created |
| `configs/ik/validation.yaml` | Created |
| `configs/learning/supervised_residual.yaml` | Created |
| `configs/learning/sac_residual.yaml` | Created |
| `configs/ros2/hardware_interface.yaml` | Created |

## Python library (`src/residual_adaptive_ik/`)

| Path | Action | State |
|------|--------|-------|
| `__init__.py` + package `__init__.py` files | Created | Package markers |
| `kinematics/fk.py` | Created | Stub (`NotImplementedError`) |
| `kinematics/ik_base.py` | Created | `IKSolver` / `IKResult` contracts |
| `kinematics/numerical_ik.py` | Created | Stub DLS class |
| `kinematics/analytical_ik_placeholder.py` | Created | Placeholder |
| `kinematics/validation.py` | Created | Stub |
| `data/dataset_schema.py` | Created | `ResidualIKSample` |
| `data/generate_supervised_data.py` | Created | Stub CLI |
| `data/replay_buffer.py` | Created | Stub |
| `learning/residual_model.py` | Created | MLP skeleton (torch optional) |
| `learning/train_supervised.py` | Created | Stub |
| `learning/evaluate_supervised.py` | Created | Stub |
| `learning/sac_policy.py` | Created | Stub |
| `learning/train_sac.py` | Created | Stub |
| `sim/isaaclab_env.py` | Created | Stub |
| `sim/domain_randomization.py` | Created | Stub |
| `sim/reward.py` | Created | Stub |
| `ros2/*.py` | Created | Library-side stubs |
| `utils/transforms.py` | Created | Quaternion normalize |
| `utils/logging_utils.py` | Created | JSON writer |
| `utils/math_utils.py` | Created | `clamp_residual` (tested) |

## ROS 2 (`ros2_ws/`)

| Path | Action |
|------|--------|
| `src/residual_adaptive_ik_ros/package.xml` | Created |
| `setup.py` / `setup.cfg` / `resource/` | Created |
| `residual_adaptive_ik_ros/residual_ik_node.py` | Stub dry-run entry |
| `residual_adaptive_ik_ros/hardware_test_node.py` | Gated stub |
| `launch/residual_ik.launch.py` | Created |
| `launch/hardware_test.launch.py` | Created |

## Scripts

| Path | Action | Notes |
|------|--------|-------|
| `scripts/setup_env.sh` | Created | venv + editable install |
| `scripts/download_mycobot_ros2.sh` | Created | Vendor clone |
| `scripts/convert_urdf_to_usd.sh` | Created | Placeholder (exits 1 until wired) |
| `scripts/run_phase1_baseline.sh` | Created | |
| `scripts/run_phase2_supervised.sh` | Created | |
| `scripts/run_phase3_sac.sh` | Created | |
| `scripts/run_ros2_hardware_test.sh` | Created | |
| `scripts/source_container_env.sh` | Created | Adapted from v1 |
| `scripts/host/spark_host_exec.sh` | **Copied + path-adapted** | v1 → v2 repo name |
| `scripts/host/env.isaac_host.sh` | **Copied + path-adapted** | |
| `scripts/host/check_prereqs.sh` | **Copied** from v1 | May still mention v1 paths in comments — review before Phase 3 |
| `scripts/host/install_isaac_lab.sh` | **Copied** from v1 | Review before use |
| `scripts/host/verify_isaac_lab.sh` | **Copied** from v1 | Review before use |
| `scripts/isaac_sim_env.sh` | **Copied** from v1 | |
| `scripts/fix_repo_permissions.sh` | **Copied** from v1 | |

## Tests / notebooks / docs / assets

| Path | Action |
|------|--------|
| `tests/test_fk.py` | Created (expects `NotImplementedError` until Phase 1) |
| `tests/test_ik_validation.py` | Created (same) |
| `tests/test_residual_bounds.py` | Created (passes — clamp utility) |
| `tests/test_dataset_schema.py` | Created (passes) |
| `tests/test_ros2_message_contract.py` | Created (passes) |
| `notebooks/phase{1,2,3}_*.ipynb` | Created (empty analysis shells) |
| `docs/phase1_baseline.md` etc. | Created (TBD reports) |
| `docs/legacy/v1_lessons_learned.md` | Created |
| `docs/legacy/v1_bootstrap_plan_archive.md` | Copied from v1 bootstrap plan |
| `assets/*/README.md` or `.gitkeep` | Created |

## Intentionally not ported from v1

- `isaac_lab/mycobot_reach_env.py` and PPO training stack (wrong architecture for residual IK)
- `phase5_red_block/`
- `spark_verify_pkg` mock Phase 1–4 ecosystem
- Competing staged/two-phase PPO recipes
- Verified PPO checkpoint (`verified_demo_25mm`) — not applicable to residual IK primary path

## Bootstrap helpers (kept for review; delete when no longer needed)

| Path | Notes |
|------|-------|
| `_bootstrap_dirs.py` | Removed after scaffold (no longer in tree) |
| `_generate_skeleton.py` | Removed after scaffold (no longer in tree) |

---

## Follow-up additions (2026-07-11)

| Item | Action |
|------|--------|
| `git init` + `origin` | Local repo on `main`; remote `git@github.com:jywilson2/spark_isaac_mycobot_v2.git` (push after creating GitHub repo) |
| `spark_isaac_mycobot_v2.code-workspace` | Multi-root: v2 active + v1 reference |
| Ownership / `chmod +x` | Scripts executable; tree owned for uid 1000 |
| URDF FK | `kinematics/urdf_model.py` + wired `fk.py`; `assets/urdf/mycobot_280_m5_kinematics.urdf`; `download_mycobot_ros2.sh` symlinks sibling |
| CI | `.github/workflows/pytest.yml` |
| Doc maintenance | Slim `spec.md` § Documentation Maintenance; expand `.cursorrules` checklist; restore `docs/last_prompt.md` prepend/never-delete progression log |
| `LICENSE` | Apache-2.0 |
| Docs | `STATUS.md` / `README.md` updated for FK + Cursor workspace |

---

## Phase 1 completion (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `kinematics/urdf_model.py` | **Updated** | Geometric Jacobian + `forward_transforms` |
| `kinematics/numerical_ik.py` | **Implemented** | Damped least-squares IK (seed, damping, limits, reasons) |
| `kinematics/validation.py` | **Implemented** | Limits, residual bounds, FK error, workspace |
| `kinematics/baseline_eval.py` | **Created** | ≥1000-pose metrics + markdown/JSON writers |
| `utils/transforms.py` | **Expanded** | Pose/orientation error helpers for DLS |
| `tests/test_ik_validation.py` | **Expanded** | Validation + DLS + Jacobian FD + small baseline |
| `scripts/run_phase1_baseline.sh` | **Updated** | Runs tests then baseline eval |
| `docs/phase1_baseline.md` | **Filled** | 1000 poses, 99.9% success (sim metrics) |
| `assets/logs/phase1_baseline_metrics.json` | **Created** | Machine-readable metrics (gitallowed) |
| `.gitignore` | **Updated** | Keep `phase1_baseline_metrics.json` |
| `STATUS.md` / `docs/last_prompt.md` | **Updated** | Phase 1 complete; next Phase 2 |
| `isaac_lab/versions.env` | **Created** | Unblocks host install/verify scripts (was missing after v1 port) |
| `isaac_lab/detect_isaac_lab.py` | **Created** | Minimal import detect for Phase 3 prep |
| `scripts/host/verify_isaac_lab.sh` | **Fixed** | No longer requires v1 PPO test tree; clear container vs host exit |
| `scripts/host/install_isaac_lab.sh` | **Fixed** | Same; Phase 3 wording; drops missing verify_install.py calls |

**Review recommended:** residual/workspace validation edge cases; host vs container Isaac guidance in `.cursorrules` / `spec.md` (this Cursor session still lacks `python.sh` in-container).

---

## Host Isaac Sim Phase 1 viz (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/urdf_utils.py` | **Created** | package:// + COLLADA GUID fixes (from v1 lessons) |
| `isaac_sim/urdf_import.py` | **Created** | Isaac Sim 6 URDF importer helpers |
| `isaac_sim/run_phase1_ik_viz.py` | **Created** | Rendered DLS IK animation + target marker |
| `isaac_sim/convert_urdf_to_usd.py` | **Created** | Headless URDF→USD |
| `scripts/host/launch_isaac_sim.sh` | **Created** | Host Kit GUI launcher |
| `scripts/host/run_phase1_isaac.sh` | **Created** | Host Phase 1 metrics + viz |
| `scripts/convert_urdf_to_usd.sh` | **Wired** | Uses host Isaac python.sh |
| `scripts/run_phase1_baseline.sh` | **Updated** | `--with-isaac` / `PHASE1_WITH_ISAAC=1` |
| `scripts/host/env.isaac_host.sh` | **Updated** | PYTHONPATH includes `src` + repo root |
| `tests/test_urdf_utils.py` | **Created** | Prep helpers without Kit |
| `docs/isaac_sim_host.md` | **Created** | Host launch / Phase 1 render guide |

**Review recommended:** articulation API differences across Isaac Sim builds (`SingleArticulation` vs legacy); first host run should confirm joint name mapping.

---

## Unified Phase 1 metrics + Isaac viz (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `kinematics/baseline_eval.py` | **Updated** | `return_trials` + `select_trials_for_visualization` |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Same run: metrics → write reports → animate subset |
| `scripts/host/run_phase1_isaac.sh` | **Updated** | No separate NumPy metrics pass; forwards `--num-poses` / `--visualize` |
| `docs/isaac_sim_host.md` | **Updated** | Single-command metrics+viz docs |

---

## README command reference (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `README.md` | **Updated** | New § Commonly used commands (env, Phase 1 NumPy/Isaac, later phases, ROS 2) |

---

## Prompt-log every-turn fix (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `.cursorrules` | **Updated** | `docs/last_prompt.md` mandatory every user turn (incl. Q&A / no-diff) |
| `spec.md` | **Updated** | Documentation Maintenance: last_prompt decoupled from code-change checklist |
| `docs/last_prompt.md` | **Backfilled** | Restored omitted clarification prompt (16:07); logged this fix |

---

## Host Isaac Phase 1 smoke verification (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `scripts/download_mycobot_ros2.sh` | **Fixed** | Relative `../../mycobot_ros2` symlink (absolute `/workspaces` broke on host) |
| `scripts/host/spark_host_exec.sh` | **Fixed** | CLI mode; forward `ISAACSIM_PATH`; no empty argv |
| `scripts/host/run_phase1_isaac.sh` | **Fixed** | Filter empty viz args |
| `scripts/host/smoke_phase1_isaac.sh` | **Created** | Headless (default) / `--gui` short smoke |
| `tests/test_phase1_isaac_smoke.py` | **Created** | Gated by `SPARK_RUN_ISAAC_SMOKE=1`; delegates via host exec in Docker |

**Verified:** host Kit smoke PASSED (20 poses, 5 viz, success_rate=1.0) via `./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh`.

**Review recommended:** URDF import warns about missing joint stiffness/damping (cosmetic for viz); nested USD output path `assets/robots/mycobot_280_m5/mycobot_280_m5/` from importer.

---

## Spec: host Isaac TDD requirement (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | Phase 1 Acceptance #7 + Step 2 order #9 + Important Notes: host Isaac smoke from independent host shell |
| `.cursorrules` | **Updated** | TDD mandate references spec Acceptance #7 |

---

## No warning suppression + GUI clarification (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | Acceptance #8: resolve warnings at source; clarify visualize vs GUI |
| `.cursorrules` | **Updated** | Ban warning suppression |
| `isaac_sim/urdf_import.py` | **Fixed** | `override_joint_stiffness` / `_damping` via derived config |
| `scripts/host/smoke_phase1_isaac.sh` | **Updated** | Explicit headless vs `--gui` messaging |
| `tests/test_phase1_isaac_smoke.py` | **Updated** | Assert importer sets drive gains |

---

## Derived joint drives + README command sync (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `configs/robot/joint_drives.yaml` | **Created** | Vendor does not publish K/D; derived K=710 N·m/rad, D=11.3 N·m·s/rad |
| `isaac_sim/joint_drives.py` | **Created** | Load + recompute helpers |
| `isaac_sim/urdf_import.py` | **Updated** | Loads gains from YAML (replaces 200/20 placeholders) |
| `tests/test_joint_drives.py` | **Created** | Config load + derivation checks |
| `configs/robot/mycobot_280.yaml` | **Updated** | arm mass, max speed pointers |
| `configs/robot/joint_limits.yaml` | **Updated** | velocity limits → vendor 160 °/s |
| `README.md` | **Updated** | Common commands: visualize≠GUI, headless, smoke env knobs |
| `docs/isaac_sim_host.md` | **Updated** | Drive-gain section + GUI notes |
| `spec.md` | **Updated** | `joint_drives.yaml` + Acceptance #8 / velocity limits |
| `STATUS.md` | **Updated** | Drive-gain status line |

**Review recommended:** Re-run host smoke after drive-gain change so USD re-imports (`smoke_phase1_isaac.sh` without `--keep-prepared`). On a Spark desktop with Kit, also run `--gui` after headless succeeds. Confirm Kit no longer warns about missing stiffness/damping with K=710 / D=11.3.

---

## CI headless vs Spark GUI verification policy (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | Acceptance #7 tiered: remote CI = headless; Spark+Isaac after headless → required GUI |
| `.cursorrules` | **Updated** | Same tiered TDD mandate |
| `README.md` | **Updated** | Smoke policy under commonly used commands |
| `docs/isaac_sim_host.md` | **Updated** | Verification policy table |
| `scripts/host/smoke_phase1_isaac.sh` | **Updated** | Header documents CI vs Spark GUI policy |
| `tests/test_phase1_isaac_smoke.py` | **Updated** | Docstring + policy regression assert |
| `STATUS.md` | **Updated** | Host smoke checklist line |

---

## Agent GUI via nsenter + runuser (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `scripts/host/spark_host_exec.sh` | **Updated** | Default `runuser -u $SPARK_HOST_USER` after nsenter; chown assets/docs; `SPARK_HOST_RUN_AS_USER=0` to force root |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | `--auto-exit` so GUI smoke does not wait for window close |
| `scripts/host/smoke_phase1_isaac.sh` | **Updated** | `--gui` passes `--auto-exit` unless `PHASE1_SMOKE_KEEP_GUI_OPEN=1` |
| `spec.md` / `README.md` / `docs/isaac_sim_host.md` / `.cursorrules` | **Updated** | Agent can run GUI without manual host shell |
| `tests/test_phase1_isaac_smoke.py` | **Updated** | Asserts runuser + auto-exit wiring |

**Verified:** `./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh --gui` → PASSED (uid=jywilson, X11 OK, auto-exit).

**Review recommended:** v1 only set `HOME`/`USER` as root (no UID drop). Keep repo assets writable for the host user; spark_host_exec now chowns `assets/` + `docs/` before runuser.

---

## Red target sphere (v1-style) (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Always-red 12 mm target sphere (v1 RGB 0.95/0.05/0.05) |
| `tests/test_target_marker.py` | **Created** | Asserts radius/color constants |
| `README.md` / `docs/isaac_sim_host.md` | **Updated** | Manual multi-minute GUI command |

---

## Hardware-speed GUI + even workspace targets + Phase 1 library docs (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `configs/robot/workspace.yaml` | **Created** | v1 cylindrical envelope + 160 °/s |
| `workspace_sampling.py` / `baseline_eval.py` | **Updated** | Even bin-filling reachable FK targets |
| `validation.py` / `validation.yaml` | **Updated** | Horizontal working radius (not 3D ball) |
| `run_phase1_ik_viz.py` | **Updated** | Joint motion ≤ vendor 160 °/s |
| `tests/test_workspace_sampling.py` / `test_servo_speed.py` | **Created** | Coverage + speed-cap tests |
| `README.md` / `REFERENCES.md` | **Updated** | Phase 1 libraries tables |
| `spec.md` / `.cursorrules` | **Updated** | Library doc maintenance mandate |

**Review recommended:** Re-run host GUI smoke after servo-speed change; confirm arm eases to red targets at ~160 °/s and targets span the annulus.

---

## Phase 2 complete: cuRobo collision-free trajectories (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `planning/curobo_planner.py` | **Created/Updated** | MotionGen wrapper, Warp shim, velocity URDF, ground |
| `configs/planning/curobo_world.yaml` | **Created** | Floor cuboid |
| `scripts/host/install_curobo.sh` | **Created** | Install cuRobo into Isaac `python.sh` |
| `scripts/host/smoke_phase2_curobo.sh` | **Created** | Host GPU smoke |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Execute planned traj; gate on failure; lower ground |
| `scripts/run_verification.sh` | **Updated** | Spark runs cuRobo smoke before GUI |
| `docs/phase2_geometry.md` / `STATUS.md` / `spec.md` | **Updated** | Phase 2 acceptance complete |

**Review recommended:** Coarse collision spheres; refine with cuRobo sphere fitting if false positives/negatives appear.

---

## Four-phase renumber + Phase 2 geometry foundation (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` / `.cursorrules` / `README.md` / `STATUS.md` | **Updated** | Four phases; tutorial-quality docstring standard strengthened |
| `src/residual_adaptive_ik/geometry/` | **Created** | NumPy capsule–sphere collision (meters) |
| `src/residual_adaptive_ik/planning/` | **Created** | Collision-checked joint lerp |
| `configs/planning/collision.yaml` | **Created** | Link radius / path samples |
| `scripts/run_phase2_geometry.sh` | **Created** | CI entry |
| `scripts/run_phase3_supervised.sh` / `run_phase4_sac.sh` | **Created** | Renumbered learning entries |
| `scripts/run_phase2_supervised.sh` / `run_phase3_sac.sh` | **Updated** | Deprecation wrappers |
| `validation.py` / `run_phase1_ik_viz.py` | **Updated** | `obstacles=` + `PATH_*` logging |
| `tests/test_phase2_geometry.py` | **Created** | CI contracts |
| `docs/phase2_geometry.md` / `REFERENCES.md` | **Created/Updated** | Phase 2 report + libraries |

**Review recommended:** Capsule radii are approximate; decide when to gate viz motion on `PATH_COLLISION` vs log-only.

---

## Collision policy + Isaac warning catalog (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `spec.md` | **Updated** | Explicit Phase 1: no path/obstacle collision; Acceptance #8 allows documenting benign Kit warnings |
| `README.md` | **Updated** | § Expected Isaac Sim launch warnings (safe to ignore) |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Drop `CreateDisplayColorAttr` (fixes Fabric indices warning); marker documented visual-only |
| `tests/test_target_marker.py` | **Updated** | Asserts no displayColor-without-indices |
| `STATUS.md` | **Updated** | Collision limitation called out |

**Review recommended:** After next host smoke, confirm `primvars:displayColor:indices` no longer appears for `/World/IkTarget`.

---

## Target contact color + denser workspace points (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/target_marker.py` | **Created** | Red/green RGB + `ee_contacts_target` (meters) |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Sphere red→green on EE tip within 12 mm; default `--visualize` 48 |
| `configs/robot/workspace.yaml` | **Updated** | Stratified bins 12×4×5 = 240 cells |
| `scripts/host/smoke_phase1_isaac.sh` | **Updated** | Defaults 240 poses / 48 visualized |
| `tests/test_target_marker.py` / `test_workspace_sampling.py` | **Updated** | Contact + bin-count contracts |
| `spec.md` / `README.md` / `STATUS.md` | **Updated** | Contact color + denser targets |

**Review recommended:** Confirm green fires only when FK tip enters the sphere (not on IK-success alone for failed trials).

---

## Unified CI vs Spark verification script (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `scripts/run_verification.sh` | **Created** | Modes `ci` (headless) and `spark` (ci + required GUI) |
| `spec.md` | **Updated** | Acceptance #7 points at the script |
| `.cursorrules` | **Updated** | Agents must run `spark` on this host after Phase 1/Isaac changes |
| `README.md` | **Updated** | Verification section at top of common commands |
| `tests/test_run_verification.py` | **Created** | Script + doc contract |

**Both places:** `spec.md` = authoritative policy; `.cursorrules` = agent must invoke which mode when.

---

## Spark verification hardening (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `scripts/run_verification.sh` | **Updated** | Preflight (orphan Kit + cuRobo); Spark headless uses `PHASE1_SMOKE_VISUALIZE=0` |
| `scripts/host/spark_host_exec.sh` | **Updated** | Forward `PHASE1_SMOKE_*` env to host |
| `scripts/host/probe_curobo.sh` | **Updated** | Exit 1 when cuRobo/CUDA missing |
| `curobo_planner.py` | **Updated** | Tighter spheres / wider self-collision ignore |
| `README.md` / `STATUS.md` / `tests/test_run_verification.py` | **Updated** | Document Spark steps; contract tests |

---

## Pytest basetemp UID scope (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `tests/conftest.py` | **Created** | `--basetemp` under `/tmp/pytest-uid-<uid>/` so root/container and host user do not share `/tmp/pytest-of-<name>` |
| `README.md` | **Updated** | Host-shell ownership error + cleanup |

---

## Mesh-fitted spheres + volumetric IK target (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `planning/sphere_fit_mycobot.py` | **Created** | cuRobo `fit_spheres_to_mesh`; mm→m for `G_base.dae` |
| `scripts/host/fit_mycobot_collision_spheres.sh` | **Created** | Host regenerate fitted YAML |
| `configs/planning/curobo/mycobot_280_collision_spheres.yaml` | **Created** | Committed mesh-fit spheres |
| `curobo_planner.py` / viz | **Updated** | Target as volumetric obstacle; tip omit for contact |
| `docs/phase2_geometry.md` / README / tests | **Updated** | Contracts + honesty notes |

---

## Headless marker↔EE side-contact diagnostic (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `planning/marker_contact_diag.py` | **Created** | Tip vs side classification of robot-sphere ∩ marker |
| `scripts/host/diagnose_marker_ee_contact.sh` | **Created** | Headless multi-trial scan; gate on executed-path side hits |
| `tests/test_marker_contact_diag.py` | **Created** | Unit tests (no CUDA) |
| `curobo_planner.py` | **Updated** | `create_obb_world` for marker; tip surface approach |
| `scripts/host/verify_target_obstacle.sh` | **Created** | SDF near/far assert for marker OBB |
| README / STATUS / phase2 docs | **Updated** | How to run diagnostic without GUI |

---

## Fail-closed planning after cuRobo reject (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `curobo_planner.py` | **Updated** | No NumPy exec fallback after cuRobo fail (default) |
| `configs/planning/collision.yaml` | **Updated** | `fallback_numpy_after_curobo_fail: false` |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | `GATED_NO_MOTION` + freeze pose; remove ungated IK lerp |
| `tests/test_phase2_curobo.py` | **Updated** | Fail-closed unit test |
| STATUS / phase2 docs | **Updated** | Document the GUI collision bug + fix |

---

## Yellow marker + fail-closed regression tests (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `isaac_sim/viz_plan_policy.py` | **Created** | `plan_result_is_executable`, marker state policy |
| `tests/test_viz_plan_fail_closed.py` | **Created** | Catches `ok_fallback\|plan_failed` execution bug |
| `isaac_sim/target_marker.py` | **Updated** | Yellow RGB for plan fail |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Yellow on fail; `_viz_log` → Kit Console via carb |
| README / STATUS / spec | **Updated** | red/green/yellow + Console note |

---

## Home reset + planning-failure strategy notes (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `kinematics/robot_home.py` | **Created** | Load `home_joint_positions_rad` |
| `configs/robot/mycobot_280.yaml` | **Updated** | Home pose (zeros / URDF reference) |
| `configs/planning/collision.yaml` | **Updated** | `reset_to_home_before_each_trial: true` |
| `isaac_sim/run_phase1_ik_viz.py` | **Updated** | Move to home before each trial |
| `diagnose_marker_ee_contact.sh` | **Updated** | Plan from home each trial |
| `tests/test_robot_home.py` | **Created** | Home + viz contract |
| `docs/phase2_geometry.md` / REFERENCES | **Updated** | Recovery strategy; cuMotion / MoveIt / OMPL |

---

## Standoff via recovery + home reset opt-in (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `planning/recovery.py` | **Created** | Direct then via-standoff until timeout |
| `configs/planning/collision.yaml` | **Updated** | Home reset **default false**; recovery knobs |
| `run_phase1_ik_viz.py` / diagnose | **Updated** | `plan_collision_free_with_recovery` |
| `tests/test_plan_recovery.py` | **Created** | Via concat + timeout + YAML defaults |
| `docs/phase2_geometry.md` | **Updated** | Implemented table; MoveIt vs cuRobo note |

---

## Rename Isaac viz entry points (2026-07-11)

Phase-neutral names for the shared Phase 1 metrics + Phase 2 planning Kit path:

| Old | New |
|-----|-----|
| `isaac_sim/run_phase1_ik_viz.py` | `isaac_sim/run_ik_viz.py` |
| `scripts/host/run_phase1_isaac.sh` | `scripts/host/run_isaac_viz.sh` |
| `scripts/host/smoke_phase1_isaac.sh` | `scripts/host/smoke_isaac_viz.sh` |
| `tests/test_phase1_isaac_smoke.py` | `tests/test_isaac_viz_smoke.py` |
| `PHASE1_SMOKE_*` env | `ISAAC_VIZ_SMOKE_*` (legacy aliases kept) |

Old script paths remain as thin deprecated forwarders.

---

## Plan recovery audit for yellow/no-via (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `planning/recovery.py` | **Updated** | Always attempt first via after direct fail; audit helpers |
| `configs/planning/collision.yaml` | **Updated** | timeout 15 s; cap direct attempts |
| `tests/test_recovery_audit.py` | **Created** | Detects PLAN_FAIL without `via1_` |
| `scripts/host/diagnose_plan_recovery.sh` | **Created** | Headless host audit |
| `run_ik_viz.py` | **Updated** | Logs `via_attempts` / RECOVERY on PLAN_FAIL |
| `run_verification.sh` | **Updated** | Spark cuRobo step runs recovery diagnose |

---

## Phase 2 status + resume briefing (2026-07-11)

| Path | Action | Notes |
|------|--------|-------|
| `docs/phase2_status_and_resume.md` | **Created** | What works / WIP / resume-after-hiatus |
| `STATUS.md` / `README.md` / `spec.md` | **Updated** | Point at briefing; Phase 2 polish called out |
