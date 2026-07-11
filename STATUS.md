# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

**Phase 1 classical IK baseline is complete.** URDF forward kinematics, damped least-squares numerical IK, and deterministic validation are implemented and covered by pytest (19 passed). Baseline evaluation over **1000** workspace-filtered reachable poses reports **99.9%** success (sim/URDF metrics only — see `docs/phase1_baseline.md`). Remote is configured as `git@github.com:jywilson2/spark_isaac_mycobot_v2.git` (create the empty GitHub repo if missing, then push). Phase 2 supervised residual is next.

## Current phase

**Phase 1 complete** — next: Phase 2 supervised residual (`train_supervised.py` / data generation).

## Checklist

| Item | Status |
|------|--------|
| Repo layout / configs / stubs | Done |
| `git init` + `origin` remote | Done (push pending GitHub repo create) |
| Multi-root `.code-workspace` | Done |
| Ownership / `chmod +x` scripts | Done |
| URDF FK (vendor + kinematics asset) | Done |
| GitHub Actions `pytest` | Done |
| `LICENSE` (Apache-2.0) | Done |
| Numerical IK (DLS) + Jacobian | Done |
| Validation (`validation.yaml`) | Done |
| Phase 1 baseline ≥1000 poses | Done (`docs/phase1_baseline.md`) |
| Phase 2 / 3 / hardware | Not started |

## How to open in Cursor

**File → Open Workspace from File…** → `spark_isaac_mycobot_v2.code-workspace`  
(or open a new window on this folder). Work in the **v2** root; use the v1 folder only as reference.

## Environment notes

```bash
source scripts/source_container_env.sh
./scripts/download_mycobot_ros2.sh   # symlink/clone vendor URDF+meshes
PYTHONPATH=src pytest tests -q
bash scripts/run_phase1_baseline.sh  # tests + ≥1000-pose metrics
```

If git reports dubious ownership:  
`git config --global --add safe.directory /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2`  
(or keep using env `GIT_CONFIG_COUNT` / `safe.directory` as in CI-less shells).

## Suggested next steps

1. Create GitHub repo `jywilson2/spark_isaac_mycobot_v2` (if empty remote) and `git push -u origin HEAD`.
2. Implement Phase 2 supervised residual data generation + MLP training stubs → real training.
3. Wire Isaac Lab residual env only after Phase 2 acceptance tests pass.
4. Keep hardware paths dry-run until `ENABLE_MYCOBOT_HARDWARE_TESTS=1`.

## Related docs

- [README.md](README.md) · [spec.md](spec.md) · [CHANGES.md](CHANGES.md) · [docs/phase1_baseline.md](docs/phase1_baseline.md) · [docs/last_prompt.md](docs/last_prompt.md)
