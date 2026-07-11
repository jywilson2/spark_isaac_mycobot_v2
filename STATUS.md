# STATUS — Residual Adaptive IK (MyCobot 280)

Last updated: **2026-07-11**

## One-paragraph summary

Fork scaffold is committed on `main` with remote `git@github.com:jywilson2/spark_isaac_mycobot_v2.git` (create the empty GitHub repo, then `git push -u origin main`). **Phase 1 forward kinematics is implemented** from the MyCobot 280 M5 URDF (NumPy + stdlib XML; CI uses `assets/urdf/mycobot_280_m5_kinematics.urdf`). Numerical IK and validation remain stubs. `pytest tests/` is green locally (9 passed). Host scripts, multi-root workspace, Apache-2.0 LICENSE, and GitHub Actions CI are in place.

## Current phase

**Phase 1 in progress** — FK done; next: DLS numerical IK + `validation.py` + baseline metrics report.

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
| Numerical IK / validation | **Not started** |
| Phase 2 / 3 / hardware | Not started |

## How to open in Cursor

**File → Open Workspace from File…** → `spark_isaac_mycobot_v2.code-workspace`  
(or open a new window on this folder). Work in the **v2** root; use the v1 folder only as reference.

## Environment notes

```bash
source scripts/source_container_env.sh
./scripts/download_mycobot_ros2.sh   # symlink/clone vendor URDF+meshes
PYTHONPATH=src pytest tests -q
```

If git reports dubious ownership:  
`git config --global --add safe.directory /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2`  
(or keep using env `GIT_CONFIG_COUNT` / `safe.directory` as in CI-less shells).

## Suggested next steps

1. Implement `kinematics/numerical_ik.py` (DLS) using FK + Jacobian.
2. Implement `kinematics/validation.py` against `configs/ik/validation.yaml`.
3. Expand tests; write `docs/phase1_baseline.md` (≥1000 poses).
4. Create GitHub repo `jywilson2/spark_isaac_mycobot_v2` and push.

## Related docs

- [README.md](README.md) · [spec.md](spec.md) · [CHANGES.md](CHANGES.md)
