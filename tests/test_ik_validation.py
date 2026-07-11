# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""IK validation contract tests — Phase 1 acceptance."""
from __future__ import annotations

import pytest


def test_validate_solution_not_implemented_yet():
    from residual_adaptive_ik.kinematics.fk import Pose
    from residual_adaptive_ik.kinematics.validation import validate_solution
    import numpy as np

    pose = Pose(position_m=np.zeros(3), quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]))
    with pytest.raises(NotImplementedError):
        validate_solution(np.zeros(6), pose, {})
