# Isaac Sim on the DGX Spark host

Phase 1 classical IK **metrics with visualization** run together on the host GPU via Isaac Sim. NumPy-only metrics (no GUI) remain available in the container.

Run these from a **native host shell** where Isaac Sim is installed (`~/isaacsim` or `ISAACSIM_PATH`).  
`isaac-ros activate` enters the ROS **Docker** container and is **not** required (and usually wrong) for this path.

## Launch Isaac Sim (GUI only)

```bash
cd /home/admin/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
export ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
./scripts/host/launch_isaac_sim.sh
```

Or: `"${ISAACSIM_PATH:-$HOME/isaacsim}/isaac-sim.sh"`

## Phase 1 metrics + visualization (one command)

Evaluates N poses for the Phase 1 report, writes JSON/Markdown, then animates a subset in Isaac Sim. Each goal is a **12 mm sphere that stays red until EE tip contact**, then turns **green**. Stratified workspace uses **240** bins (12×4×5).

The target sphere is **visual-only** (no PhysX). Phase 1 IK does **not** collision-check the path against it — see [README.md](../README.md) § Expected Isaac Sim launch warnings and [spec.md](../spec.md) § Phase 1 collision / obstacle policy. Expected Kit startup warnings are catalogued in that README section (do not mute them in code).

```bash
./scripts/download_mycobot_ros2.sh
./scripts/host/run_phase1_isaac.sh
```

**Quick test (recommended first run):**

```bash
./scripts/host/run_phase1_isaac.sh --skip-tests -- \
  --num-poses 240 --visualize 48 --hold-s 0.4
```

`--skip-tests` skips the preliminary `pytest` unit-test block only; metrics evaluation and Isaac visualization still run.

**TDD / CI host smoke** (headless Kit, short metrics+viz; from container delegates to host):

```bash
./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh
# or on host: ./scripts/host/smoke_phase1_isaac.sh
# gated pytest: SPARK_RUN_ISAAC_SMOKE=1 pytest tests/test_phase1_isaac_smoke.py -q
```

`--visualize N` animates N IK trials **inside Kit**; it does **not** open a window.  
`run_phase1_isaac.sh` opens a GUI unless you pass `--headless`.

**Verification policy**

| Context | Required |
|---------|----------|
| Remote GitHub PR / CI | Headless only (NumPy pytest ± gated Isaac smoke). **No GUI.** |
| DGX Spark host with Isaac Sim (active development) | After headless succeeds → **required** GUI: `./scripts/host/smoke_phase1_isaac.sh --gui` on a native desktop session (`DISPLAY`), **or** from the agent/container: `./scripts/host/spark_host_exec.sh ./scripts/host/smoke_phase1_isaac.sh --gui` (nsenter + **runuser** as `SPARK_HOST_USER`, `--auto-exit`) |

### Joint stiffness / damping (URDF import)

Elephant Robotics does **not** publish joint drive stiffness or damping. This repo supplies derived SI gains in `configs/robot/joint_drives.yaml` (`K = 710` N·m/rad, `D = 11.3` N·m·s/rad from payload, reach, ±0.5 mm accuracy, and arm mass). `isaac_sim/urdf_import.py` loads them into the Isaac URDF importer overrides so Kit does not warn about missing actuators. After editing the YAML, re-run import without `--keep-prepared`.

**Full ≥1000-pose acceptance with rendering:**

```bash
./scripts/host/run_phase1_isaac.sh -- \
  --num-poses 1000 --visualize 48 --hold-s 0.75
```

Useful flags (after `--`):

| Flag | Meaning |
|------|---------|
| `--num-poses N` | Poses evaluated for metrics |
| `--visualize K` | Of those, how many to animate |
| `--metrics-only` | Write metrics, skip articulation animation |
| `--visualize 0` | Same as skipping animation |
| `--headless` | No GUI window |
| `--import-only` | URDF→USD only |

NumPy-only (no Isaac), from container or host:

```bash
./scripts/run_phase1_baseline.sh
```

## Path discovery

See `scripts/isaac_sim_env.sh` (`ISAACSIM_PATH` / `ISAACSIM_PYTHON_EXE`).
