# Residual Adaptive IK — MyCobot 280

Classical inverse kinematics plus **bounded residual learning** for the Elephant Robotics MyCobot 280, using Isaac Sim, Isaac Lab, and ROS 2.

```text
Classical IK provides precision.
Learning provides adaptation.
Deterministic validation provides safety.
```

**Authoritative requirements:** [spec.md](spec.md)  
**Current status:** [STATUS.md](STATUS.md) — **Phase 1 complete**; **Phase 2 foundation on `wip_phase2`** (polish in progress). Briefing: [docs/phase2_status_and_resume.md](docs/phase2_status_and_resume.md). Baseline: [docs/phase1_baseline.md](docs/phase1_baseline.md)
**Agent policy:** [.cursorrules](.cursorrules)  
**Prompt progression log:** [docs/last_prompt.md](docs/last_prompt.md)  
**References:** [REFERENCES.md](REFERENCES.md)  
**License:** [LICENSE](LICENSE) (Apache-2.0)

---

## Phase 1 libraries (how they are used)

| Library / package | Role in Phase 1 |
|-------------------|-----------------|
| **NumPy** | Core numerics: joint vectors, SE(3)/quaternion helpers, geometric Jacobian, DLS IK (`Jᵀ(JJᵀ+λ²I)⁻¹e`), stratified workspace sampling, metrics. |
| **PyYAML** | Loads `configs/robot/*.yaml`, `configs/ik/*.yaml` (limits, validation, drive gains, workspace envelope). |
| **pytest** | Unit/contract tests under `tests/` (FK, IK, validation, workspace coverage, servo-speed caps, URDF prep). |
| **Python stdlib** (`pathlib`, `argparse`, `xml.etree`, …) | URDF parse (kinematics), CLI baseline eval, logging paths. |
| **Isaac Sim / Omniverse Kit** (host only) | `SimulationApp`, URDF importer, USD stage, Articulation API for GUI/headless viz; motion capped at vendor **160 °/s**. |
| **pxr (USD)** | Sphere target marker, ground, lights, material binding in `isaac_sim/run_ik_viz.py`. |
| **Elephant Robotics `mycobot_ros2` URDF/meshes** | Vendor kinematics + visuals via `third_party/mycobot_ros2` (or `assets/urdf/` kinematics-only for NumPy CI). |

Not used for Phase 1 deployed IK: PyTorch, Isaac Lab RL, `rclpy` (ROS node is later). Optional `pin` is listed in `requirements.txt` comments only.

Authoritative links: [REFERENCES.md](REFERENCES.md) § Phase 1 implementation libraries.

---

## Phase 2 libraries (how they are used)

| Library / package | Role in Phase 2 |
|-------------------|-----------------|
| **NumPy** | Capsule–sphere distance tests; joint-space path sampling (CI default). |
| **PyYAML** | `configs/planning/collision.yaml`. |
| **pytest** | `tests/test_phase2_geometry.py` (+ validation obstacle wiring). |
| **Isaac Sim** (host viz) | Logs `PATH_OK` / `PATH_COLLISION` during Phase 1 viz loop (same Kit entry). |
| **cuRobo** (host, Apache-2.0) | GPU MotionGen + volumetric marker; fail-closed planning. |
| **MoveIt 2** (optional later, ROS) | Approach/retreat + multi-seed IK practice reference — not the residual brain. |
| **ik_seed_bank** (in-repo) | Joint-space multi-seed IK for sequential multi-target / via2 fallback. |

Authoritative links: [REFERENCES.md](REFERENCES.md) § Phase 2 implementation libraries.

---

## Design in one line

```text
q_final = q_ik + clamp(Δq)
```

The learned model never replaces the IK solver. Residuals default to **±0.5°** per joint (experimental max ±2°). Invalid outputs fall back to classical IK, refinement, or no motion.

---

## Phases

| Phase | Goal | Entry script |
|-------|------|----------------|
| **1** | Classical FK / numerical IK / validation baseline (+ optional host Isaac viz) | `./scripts/run_phase1_baseline.sh` or `./scripts/host/run_isaac_viz.sh` |
| **2** | Geometry + collision-aware joint planning | `./scripts/run_phase2_geometry.sh` |
| **3** | Supervised residual MLP under simulated mismatch | `./scripts/run_phase3_supervised.sh` |
| **4** | SAC residual RL in Isaac Lab (host GPU) | `./scripts/run_phase4_sac.sh` |
| **ROS 2** | Dry-run residual IK node; hardware opt-in | `./scripts/run_ros2_hardware_test.sh` |

---

## Quick start

```bash
cd /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
bash scripts/setup_env.sh && source .venv/bin/activate
pytest tests
bash scripts/download_mycobot_ros2.sh
bash scripts/run_phase1_baseline.sh
```

### Phase 1 with Isaac Sim (host) — metrics + visualization

One command evaluates IK metrics and animates a subset in Isaac Sim:

```bash
# On the DGX Spark host (native shell — not the Isaac ROS container)
cd /home/admin/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
./scripts/host/run_isaac_viz.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4
```

Bare Kit GUI: `./scripts/host/launch_isaac_sim.sh`  
Details: [docs/isaac_sim_host.md](docs/isaac_sim_host.md).

### Open in Cursor

**File → Open Workspace from File…** → [`spark_isaac_mycobot_v2.code-workspace`](spark_isaac_mycobot_v2.code-workspace)  
(v2 active + v1 reference). Prefer a **new window** focused on the fork.

### DGX Spark workflow

1. Optional: `isaac-ros activate` → Cursor / ROS container for editing and NumPy tests
2. For rendered Phase 1 / Isaac Lab: use a **host** terminal (outside the container) with Isaac Sim installed
3. Container: `source scripts/source_container_env.sh` for NumPy / pytest
4. Phase 1 rendered IK / Phase 3 / USD: `scripts/host/` on the host

---

## Commonly used commands

Unless noted, run from the repo root.

### Verification: CI vs Spark host

Use **`./scripts/run_verification.sh`** so CI and host GUI paths stay distinct:

```bash
# Remote GitHub PR / CI gate (pytest; no GUI)
./scripts/run_verification.sh ci
# Optional headless Isaac on a runner that has Kit:
./scripts/run_verification.sh ci --with-isaac

# DGX Spark with Isaac Sim (pytest → headless metrics → cuRobo → required GUI)
# One-time: ./scripts/host/spark_host_exec.sh ./scripts/host/install_curobo.sh
./scripts/run_verification.sh spark
```

| Mode | Where | What runs |
|------|--------|-----------|
| `ci` | Remote PR / CI | `pytest`; optional headless Isaac (`--with-isaac`) — **never GUI** |
| `spark` | DGX Spark + Isaac Sim | CI suite, then **required** `--gui` smoke |

**Push gate (Spark):** do not push to the remote until `./scripts/run_verification.sh spark` (including GUI) has passed. See [STATUS.md](STATUS.md) § Push-to-remote gate.

Policy lives in [spec.md](spec.md) Acceptance #7; agent enforcement in [.cursorrules](.cursorrules).

- **Container** = Isaac ROS / Cursor Docker shell (often entered via `isaac-ros activate`). Good for NumPy / pytest / ROS; typically **no** Isaac Sim Kit.
- **Host** = a normal DGX Spark login shell **outside** that container, where `~/isaacsim` (or your `ISAACSIM_PATH`) is installed. Use this for `scripts/host/*` and rendered Phase 1.

`isaac-ros activate` is **not** required for Phase 1 Isaac visualization; use it when you want the ROS development container / Cursor workflow.

### Expected Isaac Sim launch warnings (safe to ignore)

Phase 1 host runs (`run_isaac_viz.sh` / `smoke_isaac_viz.sh`) print many Kit lines at startup. **Do not silence them in code** ([spec.md](spec.md) Acceptance #8). Warnings **caused by this repo** must be fixed at the source. The following are **benign vendor/Kit/platform** messages seen on the DGX Spark host and may be ignored:

| Warning (substring / source) | Why it is safe to ignore |
|------------------------------|---------------------------|
| `WARNING: All log messages before absl::InitializeLog()…` + `File already exists in database: grpc/health/v1/health.proto` / `Protobuf GeneratedMessageFactory: File is already registered` | Abseil/gRPC/protobuf double-registration inside Isaac Sim’s bundled Python stack. Harmless; not emitted by this repo. |
| `[carb.windowing-glfw.gamepad] Joystick with unknown remapping… will be ignored` | Host USB wireless dongle / HID device without a Kit gamepad map. Input is ignored; sim does not need a gamepad. |
| `[isaacsim.core.simulation_manager.plugin] USD stage is not available; stage-opened subscription is deferred` | Startup race: Simulation Manager loads before our script creates a stage. Subscription completes once the stage opens; smoke still passes. |
| `[omni.log] Source: omni.hydra was already registered` | Extension reload / duplicate Hydra log source registration inside Kit. Cosmetic. |
| `[pxr.Semantics] pxr.Semantics is deprecated - please use Semantics instead` | Upstream USD/Kit Python binding deprecation; not used by Phase 1 scripts. |
| `[omni.replicator.core…] No material configuration file, adding configuration to material settings directly` | Replicator extension default when no project material config exists. Phase 1 does not use Replicator. |
| `[omni.usd] Encountered USD Warnings but USD Diagnostics are currently muted…` | Kit preference `muteUsdDiagnostics`; informational pointer to Preferences. Not an error from our assets unless a specific prim warning follows. |
| `[carb.audio.device] audio device is misconfigured…` / `failed to retrieve the capabilities` / `Using a null streamer instead` | Headless / container / nsenter sessions often lack a working ALSA device (`$HOME/.asoundrc`). Kit falls back to a null audio streamer; Phase 1 does not need sound. |
| Joystick / audio / deferred-stage lines repeating on GUI after headless | Same causes as above; expected when `run_verification.sh spark` launches Kit twice. |

**Fixed in-repo (should no longer appear):**

| Former warning | Fix |
|----------------|-----|
| Missing joint drive stiffness/damping on URDF import | `configs/robot/joint_drives.yaml` + importer overrides |
| `[omni.fabric.plugin] primvars:displayColor:indices not found for path /World/IkTarget` | Target sphere uses **UsdPreviewSurface only** (no `CreateDisplayColorAttr` without indices) |

If a **new** warning appears that names this repo’s prims, URDF, or Python modules, treat it as a bug: fix the asset/script, do not filter the log.

**Collision note:** Phase 2 uses a volumetric 12 mm marker (cuRobo OBB). On plan OK the tip approaches the **surface** and the sphere turns **green** on contact. On plan fail: timed **standoff via-waypoint** recovery + multi-seed IK bank, then fail-closed (yellow, no motion). Home is applied **once** at viz start; per-trial home (`reset_to_home_before_each_trial` / `--reset-to-home`) is the independent-episode gate — use `--no-reset-to-home` for sequential multi-target (spec.md). See [docs/phase2_geometry.md](docs/phase2_geometry.md).

**Kit Console vs host terminal:** `print` goes to the host shell. Plan status lines are also mirrored with `carb.log_*` so they appear under Isaac Sim **Window → Console** (set the filter to Info/Verbose). That is not a live tee of the entire host terminal — only messages the viz process logs.

**Headless marker↔EE side contact** (no Kit GUI — classifies tip vs side sphere hits along planned paths):

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/diagnose_marker_ee_contact.sh --num-trials 24 --seed 0
```

Exit 1 only if an *executed* path has side contact. JSON/MD: `assets/logs/marker_ee_contact_diag.json`, `docs/marker_ee_contact_diag.md`.

### Environment and assets

#### `./scripts/setup_env.sh`

```bash
bash scripts/setup_env.sh && source .venv/bin/activate
```

Creates a local Python venv, installs `requirements.txt`, and editable-installs the package.  
**Typical use:** first-time setup on a machine (or container) before `pytest` / Phase 1 NumPy work.

#### `source scripts/source_container_env.sh`

```bash
source scripts/source_container_env.sh
```

Sets `SPARK_REPO_ROOT`, `PYTHONPATH` (includes `src/`), and ROS DDS defaults for the Isaac ROS container when `~/.bashrc` is not writable.  
**Typical use:** start of every Cursor/container terminal session before running tests or Phase 1 metrics without Isaac.

#### `./scripts/download_mycobot_ros2.sh`

```bash
./scripts/download_mycobot_ros2.sh
```

Symlinks or clones Elephant Robotics `mycobot_ros2` so the full MyCobot 280 M5 URDF **and meshes** are available under `third_party/mycobot_ros2` (relative symlink preferred so host and container share the same tree).  
**Typical use:** before Isaac Sim import / rendered Phase 1 (meshes required). CI can still use the kinematics-only URDF under `assets/urdf/` for NumPy FK/IK.

#### `./scripts/isaac_sim_env.sh`

```bash
./scripts/isaac_sim_env.sh
# or: source scripts/isaac_sim_env.sh && require_isaac_python
```

Resolves `ISAACSIM_PATH` / `ISAACSIM_PYTHON_EXE` by probing common install locations.  
**Typical use:** confirm Isaac Sim is discoverable on the host; debug “python.sh not found” before launching Kit.

---

### Testing and Phase 1 (NumPy — container or host)

#### `pytest tests`

```bash
export PYTHONPATH=src:.
pytest tests -q
```

Runs the full unit suite (FK, DLS IK, validation, URDF prep helpers, contracts). No hardware.  
On a **Spark / Isaac ROS container** host, also auto-runs the **GUI Isaac viz smoke** (`test_isaac_viz_gui_smoke`, ~3 min). Opt out: `SPARK_RUN_ISAAC_GUI_SMOKE=0`. Remote CI stays headless-only.  
**Typical use:** after any kinematics, planning, or config change; local Spark TDD default.

If a host shell shows `OSError: The temporary directory /tmp/pytest-of-<user> is not owned...`, a prior root/container run created that directory. `tests/conftest.py` scopes basetemp to `/tmp/pytest-uid-<uid>/` automatically. One-time cleanup if needed:

```bash
rm -rf /tmp/pytest-of-"$USER"   # only when that dir is owned by root
pytest tests -q
```

#### `./scripts/run_phase1_baseline.sh`

```bash
./scripts/run_phase1_baseline.sh
# optional: PHASE1_N_POSES=1000 PHASE1_SEED=0 ./scripts/run_phase1_baseline.sh
```

Runs Phase 1 unit tests, then evaluates ≥1000 reachable poses with classical DLS IK and writes `docs/phase1_baseline.md` + JSON metrics. **No GUI.**  
**Typical use:** acceptance / regression of classical IK accuracy in sim (URDF-FK metrics only). Prefer this in the container when you do not need visualization.

Pass-through to the host Isaac path:

```bash
PHASE1_WITH_ISAAC=1 ./scripts/run_phase1_baseline.sh
# or: ./scripts/run_phase1_baseline.sh --with-isaac
```

---

### Phase 1 with Isaac Sim (host)

#### `./scripts/host/launch_isaac_sim.sh`

```bash
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
./scripts/host/launch_isaac_sim.sh
```

Starts the Isaac Sim Kit GUI (`isaac-sim.sh`) on the host. Does **not** run IK metrics or load this project’s scene by itself.  
**Typical use:** interactive Omniverse work, manual USD inspection, or verifying that Kit launches before project scripts.

Equivalent one-liner:

```bash
"${ISAACSIM_PATH:-$HOME/isaacsim}/isaac-sim.sh"
```

#### `./scripts/host/run_isaac_viz.sh`

```bash
# Quick run with a visible Kit window (host desktop / DISPLAY set)
./scripts/host/run_isaac_viz.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4

# Same metrics + in-Kit animation, no GUI window
./scripts/host/run_isaac_viz.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4 --headless

# Full Phase 1 acceptance with GUI + subset animation
./scripts/host/run_isaac_viz.sh -- \
  --num-poses 1000 --visualize 48 --hold-s 0.75
```

**Single host run:** evaluates DLS IK metrics on `--num-poses` samples, writes the Phase 1 report/JSON, imports MyCobot to USD, and animates `--visualize` trials. Each IK goal is a **12 mm sphere**: **red** pending, **green** on EE tip contact, **yellow** on plan failure. Workspace sampling uses **240** stratified cells (12×4×5).  
**Typical use:** see classical IK on the real robot mesh while producing the same metrics report as the NumPy baseline. Must run on the **host** (or with Kit mounted + `SPARK_ALLOW_CONTAINER_ISAAC=1`).

`--visualize N` means animate N trials **inside Kit**; it does **not** open a window by itself. Without `--headless`, Kit starts with a GUI (needs `DISPLAY`). Prefer `./scripts/host/smoke_isaac_viz.sh` for the short TDD path (headless by default).

**Manual GUI for several minutes** (host desktop; window stays open until you close it — no `--auto-exit`):

```bash
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
./scripts/host/run_isaac_viz.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 6
# ≈ 30 × 6 s ≈ 3 minutes of target holds, then Kit stays open for inspection
```

Useful flags after `--`: `--metrics-only`, `--visualize 0`, `--headless`, `--import-only`, `--keep-prepared`, `--auto-exit`.

See [docs/isaac_sim_host.md](docs/isaac_sim_host.md).

#### `./scripts/convert_urdf_to_usd.sh`

```bash
./scripts/convert_urdf_to_usd.sh
```

Host Isaac Sim URDF→USD conversion only (no metrics animation). Writes prepared assets under `assets/robots/`.  
**Typical use:** refresh the robot USD after mesh/URDF changes without running the full Phase 1 viz loop.

#### `./scripts/host/smoke_isaac_viz.sh`

```bash
# Host shell (default: headless Kit — no window; still animates trials in-stage):
./scripts/host/smoke_isaac_viz.sh

# Visible Isaac Sim window (host graphical session with DISPLAY):
./scripts/host/smoke_isaac_viz.sh --gui

# From Cursor/container (nsenter → host Kit; headless unless you add --gui on host):
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh
```

Short Phase 1 metrics + Phase 2 planning smoke for TDD / CI-like verification. Default is **headless** Kit (CI / remote PR gate). Env knobs: `ISAAC_VIZ_SMOKE_N_POSES`, `ISAAC_VIZ_SMOKE_VISUALIZE`, `ISAAC_VIZ_SMOKE_HOLD_S`, `ISAAC_VIZ_SMOKE_RESET_TO_HOME`, `ISAAC_VIZ_MIN_PLAN_OK_RATE` (fail if PLAN_OK rate is below threshold; YAML default `0.25`).

The IK target sphere relocates only after planning finishes (red on `PLAN_OK`, yellow after recovery fail) — not at the start of each trial.

Home reset **per trial** (independent-episode mode; YAML default on for the 1.0 gate). Sequential multi-target: `--no-reset-to-home`. Viz always moves to home **once** before the first trial:

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --reset-to-home
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui --no-reset-to-home
# or:
ISAAC_VIZ_SMOKE_RESET_TO_HOME=1 ./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui
```

**Policy:** remote GitHub PR / CI = headless only. On a **DGX Spark with Isaac Sim**, after headless succeeds, run `--gui` (agent can do this without a manual host shell):

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh --gui
```

Uses nsenter + `runuser` as `SPARK_HOST_USER` and `--auto-exit`. After changing `configs/robot/joint_drives.yaml`, re-import (do not pass `--keep-prepared`) so USD regenerates.

Gated / auto pytest Isaac smokes:

```bash
# Full suite on Spark (includes GUI smoke by default):
pytest tests -q

# Headless Kit only (explicit gate):
SPARK_RUN_ISAAC_SMOKE=1 pytest tests/test_isaac_viz_smoke.py::test_isaac_viz_host_smoke -q

# Opt out of auto GUI (e.g. while iterating NumPy-only):
SPARK_RUN_ISAAC_GUI_SMOKE=0 pytest tests -q
```

#### `./scripts/host/spark_host_exec.sh`

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_isaac_viz.sh
./scripts/host/spark_host_exec.sh ./scripts/host/run_isaac_viz.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --headless
```

From the **container**, runs a repo-relative script in the host mount namespace (`nsenter`) with `ISAACSIM_PATH` forwarded. On a native host shell, runs the script directly.  
**Typical use:** agent or Cursor workflows that must exercise host Isaac Sim without leaving the container session.

#### `./scripts/host/check_prereqs.sh`

```bash
./scripts/host/check_prereqs.sh
```

Verifies Isaac Sim python discovery and related host prerequisites.  
**Typical use:** before first Phase 1 Isaac or Phase 3 install on a new machine.

---

### Later phases and ROS 2 (stubs / gated)

#### `./scripts/run_phase2_geometry.sh`

```bash
./scripts/run_phase2_geometry.sh
```

Phase 2 CI entry: geometry / planning unit tests + NumPy path-check smoke.  
**Typical use:** after changing `geometry/` or `planning/`, or as part of `./scripts/run_verification.sh ci`.

#### `./scripts/run_phase3_supervised.sh`

```bash
./scripts/run_phase3_supervised.sh
```

Entry point for supervised residual training (`train_supervised`). Still a stub until Phase 3 is implemented.  
(`scripts/run_phase2_supervised.sh` remains a deprecation wrapper.)

#### `./scripts/run_phase4_sac.sh`

```bash
./scripts/run_phase4_sac.sh
```

Entry for SAC residual RL; intended for Isaac Lab on the **host**. Stub until Phase 4 is implemented.  
(`scripts/run_phase3_sac.sh` remains a deprecation wrapper.)

#### `./scripts/host/install_isaac_lab.sh` / `./scripts/host/verify_isaac_lab.sh`

```bash
./scripts/host/install_isaac_lab.sh
./scripts/host/verify_isaac_lab.sh
```

Clone/install Isaac Lab against the host Isaac Sim tree and run a minimal import detect. Host-only.  
**Typical use:** one-time Phase 3 prerequisite setup; re-verify after Isaac Lab upgrades.

#### `./scripts/run_ros2_hardware_test.sh`

```bash
./scripts/run_ros2_hardware_test.sh validation_only
# hardware (explicit opt-in):
ENABLE_MYCOBOT_HARDWARE_TESTS=1 ./scripts/run_ros2_hardware_test.sh
```

Builds/launches the ROS 2 residual IK package. Default is dry-run / `validation_only`.  
**Typical use:** bring-up of the ROS node pipeline; enable hardware only when the arm is ready and you intentionally set the gate env var.

---

## Safety

- Hardware requires `ENABLE_MYCOBOT_HARDWARE_TESTS=1`
- Default: dry-run / `validation_only`
- Do not claim sub-mm real-world accuracy without hardware measurement

See [STATUS.md](STATUS.md) and [CHANGES.md](CHANGES.md) for current progress.
