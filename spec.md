# spec.md — Residual Adaptive IK for MyCobot 280

## Project Title

**Residual Adaptive Inverse Kinematics for a MyCobot 280 using Isaac Sim, Isaac Lab, and ROS 2**

## Purpose

Create a three-phase research and implementation project that combines classical inverse kinematics with learning-based adaptation. The objective is to retain the accuracy, determinism, and safety of analytical or numerical IK while adding RL-style robustness to noise, calibration error, tool-frame mismatch, payload effects, and real-world motion variation.

This project targets the **Elephant Robotics MyCobot 280**, a compact 6-DOF arm with approximately 280 mm working radius, 250 g payload, and approximately ±0.5 mm repeatability. The development environment is assumed to include **NVIDIA Isaac Sim**, **Isaac Lab**, a **DGX Spark** workstation, and **ROS 2** for physical hardware control.

The project must be implemented in three phases:

1. **Phase 1 — Classical IK Baseline**
2. **Phase 2 — Supervised Residual IK Model**
3. **Phase 3 — SAC-Based Residual Reinforcement Learning**

The design principle is:

```text
Classical IK provides precision.
Learning provides adaptation.
Deterministic validation provides safety.
```

The learned system must not replace the IK solver. It must wrap around it.

---

## Non-Negotiable Design Constraints

1. **Do not train a policy that directly maps target pose to all six joint angles as the primary deployed behavior.**
   - Direct learned IK may be useful as an experiment, but it is not the core architecture.
   - The deployed path must use classical IK as the base solution.

2. **The learned model outputs a residual correction, not the complete IK solution.**

   ```text
   q_final = q_ik + Δq
   ```

3. **Residual corrections must be bounded.**
   - Start with a conservative limit of ±0.5 degrees per joint.
   - Allow configuration up to ±2 degrees per joint for experiments.
   - Never allow unbounded learned corrections.

4. **Every learned output must pass validation before execution.**
   Validation must include:
   - joint limits,
   - velocity limits,
   - Cartesian pose error,
   - self-collision if available,
   - workspace bounds,
   - optional singularity/manipulability checks.

5. **Hardware execution must have a hard fallback path.**
   If the learned residual fails validation, execute one of:
   - the unmodified classical IK solution,
   - a numerical refinement result,
   - or no motion.

6. **No RL policy is allowed to command the physical MyCobot directly during training.**
   - RL training occurs in Isaac Lab / Isaac Sim only.
   - Real robot use is for evaluation and carefully gated deployment.

---

## Target Platform Assumptions

### Robot

- Robot: Elephant Robotics MyCobot 280
- Kinematic type: 6-DOF serial manipulator
- Approximate working radius: 280 mm
- Approximate payload: 250 g
- Approximate repeatability: ±0.5 mm
- Joint range: approximately ±165 degrees for J1-J5 and approximately ±179 degrees for J6, depending on model variant
- ROS 2 package: `elephantrobotics/mycobot_ros2`
- Python hardware library: `pymycobot`

### Development Machine

- Machine: NVIDIA DGX Spark or similar NVIDIA workstation (Ubuntu 24.04 / DGX OS preferred)
- GPU acceleration available
- ROS 2 **Jazzy** preferred (Humble acceptable); Isaac ROS containerized workflow supported
- Isaac Sim installed on the **host** (e.g. `~/isaacsim` or Isaac Sim build tree)
- Isaac Lab installed on the **host** (e.g. `~/IsaacLab`)
- Python 3.10+ or environment-compatible Python version

### Simulation

- Isaac Sim for robot simulation, USD assets, ROS 2 bridge integration, and visualization
- Isaac Lab for vectorized robot learning environments and RL training (Phase 3)

### Host vs container execution (required)

| Runtime | Runs Isaac Sim / Isaac Lab? | How agents and scripts execute |
|---------|----------------------------|--------------------------------|
| **DGX Spark host** (native shell after `isaac-ros activate`) | **Yes** | `./scripts/run_phase3_sac.sh` / host Isaac tools |
| **Isaac ROS container** (Cursor attached) | **No** — no GPU sim binaries | Delegate via `scripts/host/spark_host_exec.sh` (`nsenter`) |

Do **not** assume Phase 3 failed because the container lacks `python.sh`. Override host repo path with `SPARK_HOST_REPO_ROOT` or host user with `SPARK_HOST_USER` (default `admin`) when needed.

Daily workflow: host runs `isaac-ros activate` → Cursor restores this workspace → Phase 1–2 may run in the container venv → Phase 3 Isaac Lab runs on the host.

---

## High-Level Architecture

```text
                   ┌─────────────────────────────┐
                   │ Target end-effector pose     │
                   └──────────────┬──────────────┘
                                  │
                                  v
                   ┌─────────────────────────────┐
                   │ Classical IK solver          │
                   │ analytical or numerical      │
                   └──────────────┬──────────────┘
                                  │ q_ik
                                  v
                   ┌─────────────────────────────┐
                   │ Residual model / policy      │
                   │ supervised NN or SAC policy  │
                   └──────────────┬──────────────┘
                                  │ Δq
                                  v
                   ┌─────────────────────────────┐
                   │ q_candidate = q_ik + Δq      │
                   └──────────────┬──────────────┘
                                  │
                                  v
                   ┌─────────────────────────────┐
                   │ Deterministic validation     │
                   │ FK, limits, errors, safety   │
                   └──────────────┬──────────────┘
                                  │
                ┌─────────────────┴─────────────────┐
                v                                   v
      ┌───────────────────┐              ┌────────────────────┐
      │ Execute candidate │              │ Fallback / reject   │
      └───────────────────┘              └────────────────────┘
```

---

## Repository Layout

Generate the project using this structure.

**Repository root on disk:** `spark_isaac_mycobot_v2/` (this folder). Package / layout name below is the logical project name; do not nest an extra `residual_adaptive_ik_mycobot/` directory.

```text
spark_isaac_mycobot_v2/          # residual adaptive IK for MyCobot 280
├── README.md
├── STATUS.md
├── CHANGES.md
├── spec.md
├── .cursorrules
├── pyproject.toml
├── requirements.txt
├── .gitignore
├── configs/
│   ├── robot/
│   │   ├── mycobot_280.yaml
│   │   └── joint_limits.yaml
│   ├── ik/
│   │   ├── ik_solver.yaml
│   │   └── validation.yaml
│   ├── learning/
│   │   ├── supervised_residual.yaml
│   │   └── sac_residual.yaml
│   └── ros2/
│       └── hardware_interface.yaml
├── assets/
│   ├── urdf/
│   │   └── README.md
│   ├── usd/
│   │   └── README.md
│   └── meshes/
│       └── README.md
├── src/
│   └── residual_adaptive_ik/
│       ├── __init__.py
│       ├── kinematics/
│       │   ├── __init__.py
│       │   ├── fk.py
│       │   ├── ik_base.py
│       │   ├── numerical_ik.py
│       │   ├── analytical_ik_placeholder.py
│       │   └── validation.py
│       ├── data/
│       │   ├── __init__.py
│       │   ├── dataset_schema.py
│       │   ├── generate_supervised_data.py
│       │   └── replay_buffer.py
│       ├── learning/
│       │   ├── __init__.py
│       │   ├── residual_model.py
│       │   ├── train_supervised.py
│       │   ├── evaluate_supervised.py
│       │   ├── sac_policy.py
│       │   └── train_sac.py
│       ├── sim/
│       │   ├── __init__.py
│       │   ├── isaaclab_env.py
│       │   ├── domain_randomization.py
│       │   └── reward.py
│       ├── ros2/
│       │   ├── __init__.py
│       │   ├── residual_ik_node.py
│       │   ├── mycobot_driver_adapter.py
│       │   └── safety_monitor.py
│       └── utils/
│           ├── __init__.py
│           ├── transforms.py
│           ├── logging_utils.py
│           └── math_utils.py
├── ros2_ws/
│   └── src/
│       └── residual_adaptive_ik_ros/
│           ├── package.xml
│           ├── setup.py
│           ├── resource/
│           │   └── residual_adaptive_ik_ros
│           └── residual_adaptive_ik_ros/
│               ├── __init__.py
│               ├── residual_ik_node.py
│               ├── hardware_test_node.py
│               └── launch/
│                   ├── residual_ik.launch.py
│                   └── hardware_test.launch.py
├── scripts/
│   ├── setup_env.sh
│   ├── download_mycobot_ros2.sh
│   ├── convert_urdf_to_usd.sh
│   ├── run_phase1_baseline.sh
│   ├── run_phase2_supervised.sh
│   ├── run_phase3_sac.sh
│   └── run_ros2_hardware_test.sh
├── tests/
│   ├── test_fk.py
│   ├── test_ik_validation.py
│   ├── test_residual_bounds.py
│   ├── test_dataset_schema.py
│   └── test_ros2_message_contract.py
├── notebooks/
│   ├── phase1_baseline_analysis.ipynb
│   ├── phase2_residual_analysis.ipynb
│   └── phase3_rl_analysis.ipynb
└── docs/
    ├── phase1_baseline.md
    ├── phase2_supervised.md
    ├── phase3_sac.md
    ├── sim_to_real.md
    └── safety.md
```

---

# Phase 1 — Classical IK Baseline

## Goal

Build a deterministic, testable, accurate IK baseline for the MyCobot 280. This phase establishes the accuracy standard that later learning phases must match or improve under noise and model mismatch.

## Key Principle

Phase 1 is not about machine learning. It is about creating a reliable kinematic foundation.

## Required Capabilities

1. Load MyCobot 280 joint limits and kinematic parameters.
2. Implement forward kinematics.
3. Implement numerical IK baseline.
4. Add optional analytical IK placeholder for future IKFast or closed-form integration.
5. Validate joint limits and FK pose error.
6. Generate a baseline dataset of target poses and IK solutions.
7. Produce baseline metrics.

## Files to Implement

### `src/residual_adaptive_ik/kinematics/fk.py`

Implement:

```python
def forward_kinematics(q: np.ndarray) -> Pose:
    """Return end-effector pose for 6-DOF joint vector q."""
```

Requirements:

- Accept joint angles in radians.
- Return position and quaternion orientation.
- Use a consistent base frame and tool frame.
- Include unit tests against known fixture poses.

Implementation options:

- Use a URDF parser if available.
- Use Pinocchio if installed.
- Use a simple DH model only as an initial placeholder if URDF parsing is not yet wired.

### `src/residual_adaptive_ik/kinematics/ik_base.py`

Define an abstract IK interface:

```python
class IKSolver:
    def solve(self, target_pose, seed_q=None) -> IKResult:
        raise NotImplementedError
```

`IKResult` must include:

```python
@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    reason: str
```

### `src/residual_adaptive_ik/kinematics/numerical_ik.py`

Implement a damped least-squares IK solver.

Required features:

- seed-based solving,
- configurable max iterations,
- damping factor,
- position tolerance,
- orientation tolerance,
- joint limit enforcement,
- failure reason reporting.

### `src/residual_adaptive_ik/kinematics/analytical_ik_placeholder.py`

Create an interface-compatible placeholder for analytical IK.

Purpose:

- allow later IKFast or closed-form solver integration,
- preserve project architecture,
- make it easy to compare analytical, numerical, and learned-residual results.

### `src/residual_adaptive_ik/kinematics/validation.py`

Implement:

```python
def validate_solution(q, target_pose, config) -> ValidationResult:
    """Validate joint limits, residual bounds, FK pose error, and optional safety checks."""
```

Validation must check:

- q shape is `(6,)`,
- finite values only,
- joint limits,
- max allowed residual correction,
- FK position error,
- FK orientation error,
- workspace bounds,
- optional collision flag.

## Phase 1 Metrics

Generate a report with:

- success rate over random reachable target poses,
- median position error,
- 95th percentile position error,
- median orientation error,
- 95th percentile orientation error,
- average solver iterations,
- average solve time,
- failure categories.

## Phase 1 Acceptance Criteria

Phase 1 is complete when:

1. `pytest tests/test_fk.py` passes.
2. `pytest tests/test_ik_validation.py` passes.
3. At least 1,000 random reachable target poses can be evaluated.
4. Baseline IK success/failure metrics are written to `docs/phase1_baseline.md`.
5. The solver never returns an invalid joint vector as successful.
6. All units are explicit: radians, meters, seconds.

---

# Phase 2 — Supervised Residual IK Model

## Goal

Train a supervised neural model that predicts small residual joint corrections `Δq` on top of the classical IK solution. This model should compensate for known or simulated errors, such as calibration offset, tool-frame mismatch, joint bias, and noisy target pose estimates.

## Key Principle

Phase 2 should be trained before RL. Supervised residual learning is safer, easier to debug, and provides a strong initialization for Phase 3.

## Residual Formulation

The model receives:

```text
target pose
q_ik
current joint state q_current
observed or simulated FK error
optional calibration/noise parameters
```

The model outputs:

```text
Δq
```

Final candidate:

```text
q_candidate = q_ik + clamp(Δq, residual_limits)
```

## Dataset Generation

Create simulated imperfections:

1. **Joint encoder bias**
   - Add small per-joint offsets.
   - Example range: ±0.25 degrees to ±1.0 degree.

2. **Tool-frame error**
   - Add small position offset to end-effector tool frame.
   - Example range: ±1 mm to ±5 mm.

3. **Base-frame calibration error**
   - Add small position/orientation perturbation to base transform.

4. **Target-pose noise**
   - Add Gaussian noise to target position and orientation.

5. **Payload sag approximation**
   - Add a simple downward or configuration-dependent end-effector error.
   - Keep this deliberately simple for Phase 2.

## Files to Implement

### `src/residual_adaptive_ik/data/dataset_schema.py`

Define dataset schema:

```python
@dataclass
class ResidualIKSample:
    target_position: np.ndarray
    target_quaternion: np.ndarray
    q_current: np.ndarray
    q_ik: np.ndarray
    observed_position_error: np.ndarray
    observed_orientation_error: np.ndarray
    delta_q_label: np.ndarray
    metadata: dict
```

### `src/residual_adaptive_ik/data/generate_supervised_data.py`

Generate datasets:

- training set,
- validation set,
- test set,
- stress-test set with larger perturbations.

Output format:

- `.npz` for simple local experiments,
- optional Parquet for larger datasets.

### `src/residual_adaptive_ik/learning/residual_model.py`

Implement a PyTorch model:

```python
class ResidualIKModel(nn.Module):
    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """Return bounded residual joint correction Δq."""
```

Requirements:

- MLP baseline first.
- Output dimension: 6.
- Output is bounded using `tanh` scaled by residual limits.
- No recurrent model in the first implementation.

### `src/residual_adaptive_ik/learning/train_supervised.py`

Train using a composite loss:

```text
loss = FK_pose_error(q_ik + Δq_pred, target)
     + λ1 * ||Δq_pred - Δq_label||²
     + λ2 * residual_magnitude_penalty
     + λ3 * joint_limit_penalty
```

At minimum, implement:

- MSE on `Δq`,
- validation loss,
- model checkpointing,
- CSV metrics logging.

### `src/residual_adaptive_ik/learning/evaluate_supervised.py`

Compare:

1. classical IK only,
2. IK + supervised residual,
3. IK + residual + deterministic refinement.

## Phase 2 Metrics

Report:

- position error before residual,
- position error after residual,
- orientation error before residual,
- orientation error after residual,
- residual magnitude distribution,
- validation rejection rate,
- improvement under each perturbation type.

## Phase 2 Acceptance Criteria

Phase 2 is complete when:

1. The supervised model improves median Cartesian error under simulated calibration/noise conditions.
2. The model does not degrade clean baseline performance beyond an allowed tolerance.
3. Residual corrections are always bounded.
4. Validation catches invalid outputs.
5. `pytest tests/test_residual_bounds.py` passes.
6. Results are written to `docs/phase2_supervised.md`.

---

# Phase 3 — SAC-Based Residual Reinforcement Learning

## Goal

Use Soft Actor-Critic to improve residual correction behavior in simulation, especially under noisy, randomized, and partially mismatched conditions. SAC should refine the residual model from Phase 2, not replace the IK system.

TD3 may be added later as a comparison, but SAC is the initial required algorithm.

## Key Principle

The RL action is a bounded residual correction:

```text
action = Δq
```

The action is not an absolute joint command.

## Environment Design

Create an Isaac Lab environment for residual IK correction.

Each episode:

1. Sample a reachable target pose.
2. Sample current joint state.
3. Generate a classical IK solution `q_ik`.
4. Apply randomized simulation imperfections.
5. Policy outputs `Δq`.
6. Candidate solution is validated.
7. Reward is computed from final pose accuracy, safety, smoothness, and residual size.

## Observation Space

Observation should include:

```text
target position, 3 values
target orientation, quaternion or 6D rotation representation
current joint state, 6 values
classical IK solution q_ik, 6 values
estimated position error, 3 values
estimated orientation error, 3 values
previous residual action, 6 values, optional
```

Prefer normalized observations.

## Action Space

Action:

```text
Δq ∈ R⁶
```

Bound action by config:

```yaml
max_residual_deg: 0.5
```

For experiments, allow:

```yaml
max_residual_deg: 2.0
```

## Reward Function

Implement in `src/residual_adaptive_ik/sim/reward.py`.

Suggested reward:

```text
reward =
  - w_pos * position_error_m
  - w_ori * orientation_error_rad
  - w_residual * ||Δq||²
  - w_joint_limit * joint_limit_penalty
  - w_smooth * ||Δq - Δq_prev||²
  - w_invalid * invalid_solution_penalty
  + success_bonus
```

Initial weights:

```yaml
w_pos: 100.0
w_ori: 10.0
w_residual: 0.1
w_joint_limit: 10.0
w_smooth: 0.1
w_invalid: 100.0
success_bonus: 10.0
```

Success condition:

```yaml
position_error_m: <= 0.001
orientation_error_rad: <= 0.01
```

Adjust these based on MyCobot 280 physical repeatability. Do not claim sub-millimeter real-world accuracy unless hardware tests demonstrate it.

## Domain Randomization

Implement in `src/residual_adaptive_ik/sim/domain_randomization.py`.

Randomize:

- joint bias,
- target pose noise,
- tool-frame offset,
- base-frame offset,
- payload mass within safe range,
- simple compliance/sag approximation,
- latency/noise in observations.

## Files to Implement

### `src/residual_adaptive_ik/sim/isaaclab_env.py`

Create a custom Isaac Lab environment.

Requirements:

- Vectorized environment support.
- Reset function.
- Step function.
- Observation construction.
- Reward calculation.
- Termination logic.
- Metrics logging.

### `src/residual_adaptive_ik/learning/sac_policy.py`

Implement or wrap SAC policy.

Preferred options:

- use Isaac Lab-supported RL workflows if available,
- or use Stable-Baselines3 SAC as an initial implementation,
- or use a clean PyTorch SAC implementation if avoiding external RL dependencies.

### `src/residual_adaptive_ik/learning/train_sac.py`

Training script must support:

- loading Phase 2 supervised checkpoint as optional initialization,
- training from scratch,
- evaluation-only mode,
- periodic checkpointing,
- deterministic evaluation,
- CSV/JSON logging.

## Phase 3 Metrics

Compare:

1. classical IK only,
2. IK + supervised residual,
3. IK + SAC residual,
4. IK + SAC residual + deterministic refinement.

Report:

- median position error,
- 95th percentile position error,
- median orientation error,
- 95th percentile orientation error,
- success rate,
- validation rejection rate,
- fallback rate,
- residual magnitude,
- robustness under randomized perturbations.

## Phase 3 Acceptance Criteria

Phase 3 is complete when:

1. SAC residual improves robustness under randomized perturbations compared with Phase 1.
2. SAC residual does not significantly degrade clean-target performance.
3. All learned actions are bounded.
4. The validation layer rejects unsafe outputs.
5. Evaluation results are saved to `docs/phase3_sac.md`.
6. A deployment checkpoint is exported only if it passes validation tests.

---

# ROS 2 Hardware Integration

## Goal

Expose the residual IK system as a ROS 2 node for physical MyCobot 280 control.

## ROS 2 Node

Create:

```text
ros2_ws/src/residual_adaptive_ik_ros/residual_adaptive_ik_ros/residual_ik_node.py
```

## Node Behavior

The node must:

1. Subscribe to target end-effector pose.
2. Read or receive current joint state.
3. Run classical IK.
4. Optionally run residual model.
5. Validate final joint command.
6. Publish joint command only if safe.
7. Publish diagnostics.
8. Fall back safely on invalid output.

## Suggested Topics

Subscribe:

```text
/residual_ik/target_pose        geometry_msgs/PoseStamped
/joint_states                   sensor_msgs/JointState
/residual_ik/enable_learning    std_msgs/Bool
```

Publish:

```text
/residual_ik/joint_command      trajectory_msgs/JointTrajectory
/residual_ik/diagnostics        diagnostic_msgs/DiagnosticArray
/residual_ik/status             std_msgs/String
```

## Service Interfaces

Optional services:

```text
/residual_ik/set_mode
/residual_ik/validate_pose
/residual_ik/reset_policy
/residual_ik/enable_fallback
```

Modes:

```text
baseline_only
supervised_residual
sac_residual
validation_only
```

## Hardware Safety

Before moving the robot:

1. Confirm current joint state is valid.
2. Confirm target pose is inside workspace.
3. Confirm `q_candidate` is within joint limits.
4. Confirm commanded joint delta is below configured maximum.
5. Confirm learned residual is bounded.
6. Confirm emergency stop procedure is known and available.
7. Use slow speed for all initial tests.

Default hardware test limits:

```yaml
max_joint_step_deg: 2.0
max_residual_deg: 0.5
max_cartesian_step_m: 0.005
hardware_speed_scale: 0.1
```

---

# Configuration Examples

## `configs/robot/mycobot_280.yaml`

```yaml
robot_name: mycobot_280
num_joints: 6
units:
  joint_angles: radians
  position: meters
working_radius_m: 0.280
payload_limit_kg: 0.250
nominal_repeatability_m: 0.0005
joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
```

## `configs/robot/joint_limits.yaml`

```yaml
joint_limits_deg:
  joint1: [-165.0, 165.0]
  joint2: [-165.0, 165.0]
  joint3: [-165.0, 165.0]
  joint4: [-165.0, 165.0]
  joint5: [-165.0, 165.0]
  joint6: [-179.0, 179.0]
velocity_limits_deg_s:
  joint1: 30.0
  joint2: 30.0
  joint3: 30.0
  joint4: 30.0
  joint5: 30.0
  joint6: 30.0
```

## `configs/ik/validation.yaml`

```yaml
max_residual_deg: 0.5
max_position_error_m: 0.001
max_orientation_error_rad: 0.01
reject_nan: true
reject_inf: true
enforce_joint_limits: true
enforce_workspace_radius: true
workspace_radius_m: 0.280
```

## `configs/learning/supervised_residual.yaml`

```yaml
model:
  hidden_sizes: [256, 256, 128]
  activation: relu
  output_limit_deg: 0.5
training:
  batch_size: 512
  epochs: 100
  learning_rate: 0.0003
  weight_decay: 0.00001
loss:
  lambda_delta_q: 1.0
  lambda_residual_magnitude: 0.01
  lambda_joint_limit: 10.0
```

## `configs/learning/sac_residual.yaml`

```yaml
algorithm: SAC
action_space: residual_joint_delta
max_residual_deg: 0.5
training:
  total_steps: 1000000
  batch_size: 1024
  gamma: 0.99
  tau: 0.005
  actor_learning_rate: 0.0003
  critic_learning_rate: 0.0003
  entropy_auto_tune: true
reward:
  w_pos: 100.0
  w_ori: 10.0
  w_residual: 0.1
  w_joint_limit: 10.0
  w_smooth: 0.1
  w_invalid: 100.0
  success_bonus: 10.0
success:
  position_error_m: 0.001
  orientation_error_rad: 0.01
```

---

# Testing Requirements

## Unit Tests

Implement tests for:

1. FK output shape and finite values.
2. IK result schema.
3. Joint-limit validation.
4. Residual clamping.
5. Dataset schema serialization/deserialization.
6. ROS 2 message contract.

## Integration Tests

Implement tests for:

1. random target pose IK solving,
2. IK + residual validation,
3. supervised model inference,
4. simulated environment reset/step,
5. ROS 2 node dry-run mode.

## Hardware Tests

Hardware tests must be opt-in only.

Require an explicit flag:

```bash
export ENABLE_MYCOBOT_HARDWARE_TESTS=1
```

No hardware movement should occur unless this flag is set.

---

# Cursor Implementation Instructions

Use the following process when generating code in Cursor.

## Step 1 — Create the skeleton

Create the repository layout exactly as specified. Add placeholder files with docstrings where implementation will follow.

## Step 2 — Implement Phase 1 first

Do not start RL code until Phase 1 passes tests.

Required Phase 1 order:

1. data classes,
2. transform utilities,
3. FK,
4. numerical IK,
5. validation,
6. tests,
7. baseline script,
8. baseline report.

## Step 3 — Implement Phase 2

Required Phase 2 order:

1. dataset schema,
2. perturbation generator,
3. supervised dataset generator,
4. residual MLP,
5. training script,
6. evaluation script,
7. report.

## Step 4 — Implement Phase 3

Required Phase 3 order:

1. Isaac Lab environment stub,
2. reward function,
3. domain randomization,
4. SAC training wrapper,
5. evaluation comparison,
6. export checkpoint,
7. report.

## Step 5 — Implement ROS 2 integration

Required ROS 2 order:

1. dry-run residual IK node,
2. diagnostic publisher,
3. baseline-only mode,
4. supervised residual mode,
5. SAC residual mode,
6. hardware adapter,
7. gated hardware test.

---

# Definition of Done

The project is complete when:

1. Phase 1 baseline IK works and is measured.
2. Phase 2 supervised residual improves noisy/mismatched conditions.
3. Phase 3 SAC residual improves robustness beyond Phase 1 and preferably beyond Phase 2.
4. All learned corrections are bounded.
5. Hardware execution is opt-in and safety-gated.
6. ROS 2 node can run in dry-run mode without hardware.
7. Documentation explains how to reproduce each phase.
8. The final system can run in one of these modes:

```text
baseline_only
supervised_residual
sac_residual
validation_only
```

---

# Recommended Initial Command Set

```bash
# Create and activate Python environment
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

# Run unit tests
pytest tests

# Run Phase 1
bash scripts/run_phase1_baseline.sh

# Run Phase 2
bash scripts/run_phase2_supervised.sh

# Run Phase 3
bash scripts/run_phase3_sac.sh

# Build ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run dry-run ROS 2 node
ros2 launch residual_adaptive_ik_ros residual_ik.launch.py mode:=validation_only
```

---

# Important Notes for Cursor

- Keep code modular.
- Prefer small testable files over large monolithic scripts.
- Use type hints.
- Use dataclasses for result objects.
- Keep all units explicit.
- Never silently ignore validation failures.
- Never execute hardware motion from tests by default.
- Never allow an RL policy to bypass validation.
- Store all experiment metrics in machine-readable CSV or JSON.
- Write reports in Markdown under `docs/`.
- Use deterministic seeds for repeatable experiments.
- Prefer tutorial-quality comments: explain *why*, link to this spec / README / STATUS, and relate residuals to physical joint motion on the MyCobot.
- Agent/editor policy that is not a product requirement lives in [`.cursorrules`](.cursorrules).

---

# Documentation Maintenance

Every change set that alters behavior, configs, training, or verification **must** update:

| Document | Purpose |
|----------|---------|
| [spec.md](spec.md) | Authoritative requirements (this file) |
| [README.md](README.md) | How to run the current phase |
| [STATUS.md](STATUS.md) | Operational status / blockers / next steps |
| [docs/phaseN_*.md](docs/) | Phase reports when metrics change |

Do **not** claim hardware accuracy beyond measured results. Simulation success thresholds (e.g. 1 mm position error) are **sim metrics** unless hardware tests confirm them.

---

# Relationship to Prior Project (`spark_isaac_mycobot_demo`)

This repository is a **fork / restart**, not a continuation of the v1 EE-reach PPO pipeline.

| Topic | v1 demo | This project (v2) |
|-------|---------|-------------------|
| Core idea | Learned joint deltas *replace* IK | Classical IK + bounded residual `Δq` |
| RL algorithm | PPO (RSL-RL) | SAC residual (Phase 3), after supervised Phase 2 |
| Classical IK | Prohibited in control loop | **Required** base solver |
| Host scripts | Proven `spark_host_exec` pattern | Reused under `scripts/host/` |

Operational lessons from v1 are summarized in [docs/legacy/v1_lessons_learned.md](docs/legacy/v1_lessons_learned.md). Do not port the v1 PPO reach environment as the primary architecture.

---

# Future Extensions

After the three phases are complete, consider:

1. IKFast integration if a stable analytical model is available.
2. TRAC-IK or MoveIt 2 plugin integration.
3. Branch selection among multiple IK solutions.
4. Residual correction in Cartesian space instead of joint space.
5. Online calibration estimation.
6. Real-time perception target noise experiments.
7. Sim-to-real evaluation matrix.
8. TD3 comparison against SAC.
9. Learned damping/solver-parameter policy.
10. Formal safety monitor for MoveIt Servo or ros2_control integration.
