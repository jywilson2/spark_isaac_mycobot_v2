# Residual Adaptive IK — MyCobot 280

Classical inverse kinematics plus **bounded residual learning** for the Elephant Robotics MyCobot 280, using Isaac Sim, Isaac Lab, and ROS 2.

```text
Classical IK provides precision.
Learning provides adaptation.
Deterministic validation provides safety.
```

**Authoritative requirements:** [spec.md](spec.md)  
**Current status:** [STATUS.md](STATUS.md) — **Phase 1 complete** (FK + DLS IK + validation; [baseline report](docs/phase1_baseline.md))  
**Agent policy:** [.cursorrules](.cursorrules)  
**Prompt progression log:** [docs/last_prompt.md](docs/last_prompt.md)  
**References:** [REFERENCES.md](REFERENCES.md)  
**License:** [LICENSE](LICENSE) (Apache-2.0)

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
| **1** | Classical FK / numerical IK / validation baseline | `./scripts/run_phase1_baseline.sh` |
| **2** | Supervised residual MLP under simulated mismatch | `./scripts/run_phase2_supervised.sh` |
| **3** | SAC residual RL in Isaac Lab (host GPU) | `./scripts/run_phase3_sac.sh` |
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

### Open in Cursor

**File → Open Workspace from File…** → [`spark_isaac_mycobot_v2.code-workspace`](spark_isaac_mycobot_v2.code-workspace)  
(v2 active + v1 reference). Prefer a **new window** focused on the fork.

### DGX Spark workflow

1. Host: `isaac-ros activate`
2. Cursor: open this workspace
3. Container: `source scripts/source_container_env.sh`
4. Phase 3 / USD: Isaac Sim on the **host** via `scripts/host/`

---

## Safety

- Hardware requires `ENABLE_MYCOBOT_HARDWARE_TESTS=1`
- Default: dry-run / `validation_only`
- Do not claim sub-mm real-world accuracy without hardware measurement

See [STATUS.md](STATUS.md) and [CHANGES.md](CHANGES.md) for current progress.
