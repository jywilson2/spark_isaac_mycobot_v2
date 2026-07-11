# Residual Adaptive IK — MyCobot 280

Classical inverse kinematics plus **bounded residual learning** for the Elephant Robotics MyCobot 280, using Isaac Sim, Isaac Lab, and ROS 2.

```text
Classical IK provides precision.
Learning provides adaptation.
Deterministic validation provides safety.
```

**Authoritative requirements:** [spec.md](spec.md)  
**Current status:** [STATUS.md](STATUS.md)  
**Agent policy:** [.cursorrules](.cursorrules)  
**References:** [REFERENCES.md](REFERENCES.md)

---

## Design in one line

```text
q_final = q_ik + clamp(Δq)
```

The learned model never replaces the IK solver. It wraps around it. Residuals default to **±0.5°** per joint (experimental max ±2°). Invalid outputs fall back to classical IK, refinement, or no motion.

---

## Phases

| Phase | Goal | Entry script |
|-------|------|----------------|
| **1** | Classical FK / numerical IK / validation baseline | `./scripts/run_phase1_baseline.sh` |
| **2** | Supervised residual MLP under simulated mismatch | `./scripts/run_phase2_supervised.sh` |
| **3** | SAC residual RL in Isaac Lab (host GPU) | `./scripts/run_phase3_sac.sh` |
| **ROS 2** | Dry-run residual IK node; hardware opt-in | `./scripts/run_ros2_hardware_test.sh` |

Modes: `baseline_only` · `supervised_residual` · `sac_residual` · `validation_only`

---

## Quick start

```bash
cd /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2
bash scripts/setup_env.sh
source .venv/bin/activate
pytest tests

# Optional: vendor URDF
bash scripts/download_mycobot_ros2.sh

# Phase 1 (stubs until FK/IK implemented)
bash scripts/run_phase1_baseline.sh
```

### DGX Spark / Cursor workflow

1. **Host:** `isaac-ros activate`
2. **Cursor:** open this folder (container attaches)
3. **Container terminal:** `source scripts/source_container_env.sh`
4. Phase 1–2: local `.venv` is fine  
5. Phase 3 / USD conversion: Isaac Sim on the **host** via `scripts/host/`

| Runtime | Isaac Sim / Lab? |
|---------|------------------|
| Host | Yes |
| Isaac ROS container | No — use `spark_host_exec.sh` |

---

## Repository layout (summary)

```text
configs/          Robot, IK, learning, ROS 2 YAML
src/residual_adaptive_ik/   Python library (kinematics → learning → sim)
ros2_ws/          ament_python package residual_adaptive_ik_ros
scripts/          Phase runners + host delegation
tests/            pytest (no hardware by default)
docs/             Phase reports + legacy lessons
assets/           URDF/USD/meshes/datasets/checkpoints
```

Full tree and acceptance criteria: [spec.md](spec.md).

---

## Safety

- Hardware tests require `export ENABLE_MYCOBOT_HARDWARE_TESTS=1`
- Default launches are dry-run / `validation_only`
- No RL policy commands the physical arm during training
- Do not claim sub-mm real-world accuracy without hardware measurement

---

## Documentation map

| File | Role |
|------|------|
| [spec.md](spec.md) | Requirements, phases, acceptance |
| [STATUS.md](STATUS.md) | Where we are / next steps |
| [CHANGES.md](CHANGES.md) | Scaffold change log |
| [docs/safety.md](docs/safety.md) | Safety notes |
| [docs/legacy/v1_lessons_learned.md](docs/legacy/v1_lessons_learned.md) | Lessons from `spark_isaac_mycobot_demo` |
