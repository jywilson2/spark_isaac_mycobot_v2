# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

Fresh repository scaffold for **classical IK + bounded residual learning** (supervised → SAC). Directory tree, configs, Python stubs, ROS 2 package skeleton, host-delegation scripts (ported from `spark_isaac_mycobot_demo`), and tests that exercise the residual clamp / dataset schema are in place. **Phase 1 FK / numerical IK / validation are not implemented yet** (`NotImplementedError` stubs). No training runs have been executed in this fork.

## Current phase

**Phase 0 → Phase 1 scaffolding complete.** Next engineering work: implement FK + DLS IK + validation so `pytest tests/test_fk.py` and `tests/test_ik_validation.py` pass with real assertions (not only `NotImplementedError` expectations).

## Checklist

| Item | Status |
|------|--------|
| Repo layout per [spec.md](spec.md) | Done |
| Config YAMLs | Done |
| Python package stubs under `src/residual_adaptive_ik/` | Done |
| ROS 2 package skeleton | Done (dry-run node still stubbed) |
| Host scripts (`spark_host_exec`, Isaac Lab install/verify) | Copied / path-adapted from v1 |
| Phase 1 FK / IK / validation implementation | **Not started** |
| Phase 2 supervised training | Not started |
| Phase 3 SAC / Isaac Lab env | Not started |
| Hardware dry-run ROS node | Stub only |
| Hardware motion | Blocked until gated implementation |

## Environment notes

```bash
# Container
source /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2/scripts/source_container_env.sh
whoami   # expect admin (uid 1000) on Spark workflow

# Local Python
bash scripts/setup_env.sh && source .venv/bin/activate && pytest tests
```

Isaac Sim / Isaac Lab: **host only**. See `scripts/host/`.

## Blockers

None for scaffolding. Implementation depends on:

1. MyCobot URDF availability (`scripts/download_mycobot_ros2.sh` or existing `third_party/`)
2. Host Isaac Sim path for Phase 3 / USD conversion

## Suggested next steps

1. Implement `kinematics/fk.py` against vendor URDF (or Pinocchio/DH placeholder with clear limits).
2. Implement `numerical_ik.py` (DLS) + `validation.py`.
3. Replace stub tests with real FK fixture / IK metric tests; write `docs/phase1_baseline.md`.
4. Only then generate supervised datasets (Phase 2).

## Related docs

- [spec.md](spec.md) — requirements
- [README.md](README.md) — how to run
- [CHANGES.md](CHANGES.md) — scaffold inventory
- [docs/legacy/v1_lessons_learned.md](docs/legacy/v1_lessons_learned.md)
