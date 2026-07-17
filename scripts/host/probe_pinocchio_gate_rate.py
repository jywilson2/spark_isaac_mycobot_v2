#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO / "src"), str(REPO)]

import numpy as np

from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.numerical_ik import load_joint_limits_rad
from residual_adaptive_ik.planning.dexterity import contact_pose_is_dexterous


def main() -> int:
    lo, hi = load_joint_limits_rad()
    rng = np.random.default_rng(0)
    ok = ori = pos = 0
    n = 0
    for _ in range(80):
        q = rng.uniform(lo, hi)
        t = forward_kinematics(q).position_m
        r = float(np.linalg.norm(t))
        if not (0.16 < r < 0.24):
            continue
        n += 1
        res = contact_pose_is_dexterous(t, 0.02, n_seeds=12, backend="pinocchio")
        if res.classification == "ok":
            ok += 1
        elif res.classification == "unreachable_orientation":
            ori += 1
        else:
            pos += 1
    print(f"in_band_samples={n} ok={ok} ori_skip={ori} pos_skip={pos}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
