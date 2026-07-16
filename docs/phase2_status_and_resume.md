# Phase 2 — What works, what’s next, how to resume

**Branch:** `wip_phase2`  
**Last updated:** 2026-07-12  
**Units:** meters, radians, seconds  

This is the operational briefing for Phase 2 (geometry + collision-aware planning). Authoritative requirements remain in [spec.md](../spec.md) § Phase 2. Day-to-day commands: [README.md](../README.md), [STATUS.md](../STATUS.md).

---

## What works (shipped on `wip_phase2`)

| Area | Status | Notes |
|------|--------|-------|
| NumPy capsule–sphere + ground checks | **Done** | CI / no-GPU path (`planning/joint_path.py`) |
| cuRobo `MotionGen` on DGX Spark | **Done** | Host GPU; Apache-2.0; ground cuboid in world YAML |
| Mesh-fitted collision spheres | **Done** | `fit_mycobot_collision_spheres.sh` + committed YAML |
| Volumetric 12 mm IK marker as obstacle | **Done** | Sphere → OBB via `WorldConfig.create_obb_world` (raw spheres alone are ignored by cuRobo’s PRIMITIVE checker) |
| Tip plans to marker **surface** | **Done** | Avoids forcing flange through marker center |
| Fail-closed planning | **Done** | No NumPy exec fallback after cuRobo reject; yellow marker; freeze pose |
| Deferred marker relocate | **Done** | Sphere moves only on PLAN_OK (red) or after recovery fail (yellow) — not before planning |
| Min PLAN_OK rate smoke gate | **Done** | `min_plan_ok_rate: 1.0` (100% required; verified 48/48 GUI) |
| Contact-leg tip omit (`omit_tip_links`) | **Done** | Direct / via2 use second MotionGen without flange spheres |
| Timeout recovery + partial via exec | **Done** | Loop until `plan_recovery_timeout_s` (**90 s**); execute via1 mid-budget; yellow only after timeout |
| Far recovery vias | **Done** | Standoffs ordered nearest → farthest; `plan_recovery_min_standoff_travel_m: 0.01` |
| Tip-face green contact | **Done** | Middle of tip pad on approach pierce; side grazes invalid |
| GUI in automated pytest (Spark) | **Done** | `test_isaac_viz_gui_smoke` auto-runs; opt out `SPARK_RUN_ISAAC_GUI_SMOKE=0` |
| Standoff via-waypoint **planning** recovery | **Done** | Clearances + lateral yaw; near-surface vias; INVALID_START escape toward home |
| Headless recovery audit | **Done** | `diagnose_plan_recovery.sh` — fails if PLAN_FAIL has zero `via1_` |
| Marker↔EE side-contact diagnostic | **Done** | `diagnose_marker_ee_contact.sh` (tip vs side sphere hits) |
| Isaac viz rename | **Done** | `smoke_isaac_viz.sh` / `run_isaac_viz.sh` / `run_ik_viz.py` |
| Home reset | **Done (opt-in per trial)** | Once at viz start always; per-trial only via `--reset-to-home` (GUI smoke does **not**) |
| Spark verification pipeline | **Done** | `./scripts/run_verification.sh spark` |
| Kit Console mirroring | **Done** | `carb.log_*` for PLAN_* lines (not a full host-terminal tee) |

### Marker colors (GUI)

| Color | Meaning |
|-------|---------|
| Red | Pending approach (plan OK path, tip not in contact) |
| Green | EE tip-face contact with marker surface |
| Yellow | Plan failed **or** PLAN_OK without surface contact; **arm does not move** |

The marker **does not teleport** at the start of a trial. It relocates only when planning finishes: red + EE motion on `PLAN_OK`, or yellow after recovery timeout/exhaustion on `PLAN_FAIL`.

**Contact is required for success:** A trial that gets PLAN_OK but where the tip never reaches the sphere surface (`MARKER_NO_CONTACT`) is reclassified as PLAN_FAIL (yellow). The rate gate enforces both planning and surface contact.

### Honest limits (known)

- Yellow + motionless is **expected** when every plan (direct + vias) fails. Recovery retries are **planning-only** until a full path succeeds — the arm does not animate failed via legs.
- With contact-leg tip omit, GUI smoke without per-trial home reset recently measured PLAN_OK well above the 0.25 gate (path-dependent starts still produce some `INVALID_START_*` / recovery fails — intentional for recovery testing).
- Marker turns **green** when the tip-face center reaches the sphere surface (contact legs plan tip onto the surface with tip spheres omitted).
- Viz / smoke **fails** (exit ≠ 0) when `PLAN_OK/(OK+FAIL) < min_plan_ok_rate` (default 0.25). Trials with no surface contact are counted as failures. Use `ISAAC_VIZ_MIN_PLAN_OK_RATE=0` only when debugging metrics without gating.
- Fitted spheres approximate meshes; not exact mesh–mesh contact.
- Simulation thresholds (e.g. 1 mm) are sim metrics — do not claim sub-mm hardware accuracy without gated hardware tests.

---

## Still in development / not done

| Item | Status | Intent |
|------|--------|--------|
| Higher PLAN_OK rate under volumetric marker | **Mostly done** | Contact tip-omit + home-reset smoke ≈0.96; further polish optional |
| Execute partial recovery (move to standoff, then final approach) | **Not started** | Would make recovery visible in GUI; currently plan-then-execute-only-if-complete |
| Lateral / alternate IK seed retries | **Not started** | Extra recovery strategies beyond standoff vias |
| MoveIt 2 / Isaac ROS cuMotion integration | **Not started** | Optional later; cuRobo remains preferred for this Isaac setup |
| Merge `wip_phase2` → `main` | **Pending** | When verification is green and docs agreed |
| Phase 3 supervised residual `Δq` | **Not started** | Bounded residual on classical IK — **not** a substitute for collision-free planning |
| Phase 4 SAC residual (Isaac Lab only) | **Not started** | No RL commands to physical MyCobot during training |
| ROS 2 dry-run → gated hardware | **Not started** | Requires `ENABLE_MYCOBOT_HARDWARE_TESTS=1` for motion |

---

## Resume development after a long hiatus

### 1. Environment

```bash
# Prefer: Isaac ROS / Cursor container for editing + pytest
cd /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2   # or host clone path
git fetch origin
git checkout wip_phase2
git pull --rebase origin wip_phase2

# Host Isaac Sim path (Spark)
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
# Optional: SPARK_HOST_REPO_ROOT / SPARK_HOST_USER if nsenter path/user differ
```

Read first: [STATUS.md](../STATUS.md), this file, [docs/phase2_geometry.md](phase2_geometry.md), [spec.md](../spec.md) § Phase 2.

### 2. Fast confidence checks (no Kit GUI)

```bash
# Container or host venv
PYTHONPATH=src:. python3 -m pytest tests -q

# Phase 2 NumPy-only
./scripts/run_phase2_geometry.sh
```

### 3. Host GPU / cuRobo (required for planning)

```bash
# One-time if missing
./scripts/host/spark_host_exec.sh ./scripts/host/install_curobo.sh
./scripts/host/spark_host_exec.sh ./scripts/host/probe_curobo.sh

# Optional: regenerate fitted spheres after mesh/URDF changes
./scripts/host/spark_host_exec.sh ./scripts/host/fit_mycobot_collision_spheres.sh

./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase2_curobo.sh
./scripts/host/spark_host_exec.sh ./scripts/host/verify_target_obstacle.sh
./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_plan_recovery.sh --num-trials 16
```

### 4. Full Spark gate (do **not** pipe through `head`/`tail`)

```bash
./scripts/run_verification.sh spark
```

Preflight refuses orphan `run_ik_viz.py` / Kit processes. Headless metrics → cuRobo + recovery audit → GUI smoke.

### Push gate (required before `git push`)

On Spark, **never push** until GUI smoke has passed for the change set:

```bash
./scripts/run_verification.sh spark   # includes required GUI
```

A green NumPy `pytest` alone (or `SPARK_RUN_ISAAC_GUI_SMOKE=0`) is not enough to push. Canonical short form: [STATUS.md](../STATUS.md) § Push-to-remote gate.

### 5. Interactive GUI

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui
# Independent starts:
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
```

Look for: `Phase 2 planner: cuRobo MotionGen`, `PLAN_OK` / `PLAN_FAIL` with `via_attempts=N`, marker red→green or yellow.

### 6. Suggested next engineering (before Phase 3)

1. Drive up PLAN_OK rate (home-reset A/B, standoff clearances, start-state validation).
2. Optionally execute standoff leg when via1 succeeds and via2 fails (visible recovery).
3. Keep hardware dry-run; residuals stay bounded and validation-gated.
4. Merge `wip_phase2` → `main` when ready.

### 7. Architecture reminders (non-negotiable)

- Deployed path: `q_final = q_ik + clamp(Δq)` — classical IK base; learning only residuals.
- Never ship pose → full 6-DOF joints as primary hardware behavior.
- No RL policy commanding the physical MyCobot during training.
- Host vs container: Kit / cuRobo on the **host**; use `spark_host_exec.sh` from the container.

---

## Key commands cheat sheet

| Goal | Command |
|------|---------|
| GUI smoke | `./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui` |
| Full Spark verify | `./scripts/run_verification.sh spark` |
| Recovery audit | `./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_plan_recovery.sh` |
| Marker side-contact | `./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_marker_ee_contact.sh` |
| Unit tests | `PYTHONPATH=src:. python3 -m pytest tests -q` |
