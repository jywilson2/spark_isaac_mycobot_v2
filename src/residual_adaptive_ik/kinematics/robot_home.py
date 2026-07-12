# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Robot-level config helpers (home pose, etc.) — no Kit required.

Units: joint angles in radians. See ``configs/robot/mycobot_280.yaml``.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROBOT_YAML = _REPO_ROOT / "configs" / "robot" / "mycobot_280.yaml"


def load_robot_config(path: Path | None = None) -> dict[str, Any]:
    cfg_path = Path(path) if path is not None else DEFAULT_ROBOT_YAML
    with cfg_path.open(encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


def load_home_joint_positions_rad(path: Path | None = None) -> np.ndarray:
    """Return the configured home / ready joint vector (6,), radians.

    Used to reset the arm at the start of each independent planning trial so
    success rates are not biased by the previous goal configuration.
    """
    cfg = load_robot_config(path)
    q = np.asarray(cfg.get("home_joint_positions_rad", [0.0] * 6), dtype=float).reshape(-1)
    if q.size != 6:
        raise ValueError(f"home_joint_positions_rad must have length 6, got {q.size}")
    return q
