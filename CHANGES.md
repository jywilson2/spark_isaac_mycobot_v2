# CHANGES — Scaffold inventory (2026-07-11)

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
- `last_prompt.md` retention policy
- Verified PPO checkpoint (`verified_demo_25mm`) — not applicable to residual IK primary path

## Bootstrap helpers (kept for review; delete when no longer needed)

| Path | Notes |
|------|-------|
| `_bootstrap_dirs.py` | One-shot dir creator used during scaffold |
| `_generate_skeleton.py` | One-shot file generator used during scaffold |
