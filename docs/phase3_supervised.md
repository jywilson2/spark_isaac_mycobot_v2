# Phase 3 — Supervised Residual IK (report)

**Status:** Scaffold + first trained MLP complete (2026-07-16).  
**Branch:** `wip_phase3`  
**Deployed path:** `q_final = q_ik + clamp(Δq)` with `Δq` bounded to ±0.5°.

## What was built

| Component | Path | Notes |
|-----------|------|-------|
| Dataset schema | `data/dataset_schema.py` | `ResidualIKSample` |
| Dataset generator | `data/generate_supervised_data.py` | joint bias, tool/base frame, target noise, payload sag |
| Residual MLP | `learning/residual_model.py` | tanh-scaled ±0.5° |
| Training | `learning/train_supervised.py` | obs standardization, MSE + light mag penalty, CSV + checkpoint |
| Evaluation | `learning/evaluate_supervised.py` | IK / oracle / MLP comparison |
| Host train script | `scripts/host/train_supervised_residual.sh` | Isaac Sim host python (torch) |
| Config | `configs/learning/supervised_residual.yaml` | |

## Metrics (test split, n=200, seed=44)

| Mode | Median tip error vs true (m) | Median improvement (m) | Residual \|Δq\| median (rad) | Val reject rate |
|------|------------------------------|------------------------|-------------------------------|-----------------|
| IK only | 0.00200 | 0 | 0 | 0.02 |
| IK + oracle residual | 0.00102 | **0.00097** | 0.0151 | 0.02 |
| IK + MLP residual | 0.00160 | **0.00039** | 0.0073 | 0.025 |

Oracle shows the upper bound for this label definition (~1 mm median tip improvement). The MLP recovers a substantial fraction of that (~0.4 mm) after observation standardization and a light magnitude penalty.

Full JSON: `assets/logs/phase3_supervised_eval.json`  
Checkpoint: `assets/checkpoints/supervised_residual/best.pt`  
Train CSV: `assets/checkpoints/supervised_residual/train_metrics.csv`

## How to reproduce

```bash
# On DGX Spark host (Isaac python has torch):
./scripts/host/spark_host_exec.sh ./scripts/host/train_supervised_residual.sh 100

# Container / CI (no torch): generate + oracle eval only
PYTHONPATH=src:. python3 -m residual_adaptive_ik.data.generate_supervised_data \
  --train 200 --val 50 --test 50 --stress 50
PYTHONPATH=src:. python3 -m residual_adaptive_ik.learning.evaluate_supervised
pytest tests/test_generate_supervised_data.py tests/test_evaluate_supervised.py \
  tests/test_residual_bounds.py tests/test_residual_model.py -q
```

## Acceptance checklist (`spec.md` Phase 3)

| Criterion | Status |
|-----------|--------|
| Improves median Cartesian error under sim calibration/noise | **Yes** (MLP +0.39 mm median on test) |
| Does not degrade clean baseline beyond tolerance | Residual clamped ±0.5°; validation reject ≤3% |
| Residuals always bounded | `tanh` × limit + `clamp_residual` |
| Validation catches invalid outputs | `validate_solution` before accept |
| `pytest tests/test_residual_bounds.py` | Pass |
| Report written | This file |

## Honest limits / next

- Labels are `clamp(q_true − q_ik)`; for tip-frame noise the oracle cannot fully recover with ±0.5°.
- MLP still trails oracle; next: FK pose-error loss term, larger train set, stress-set reporting.
- Phase 4 SAC should initialize from this checkpoint — do not command hardware during RL.
