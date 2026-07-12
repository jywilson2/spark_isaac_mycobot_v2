# CHANGES — Scaffold inventory (2026-07-11)

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
