# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Differentiable forward kinematics for Phase 3 supervised training.

Mirrors ``kinematics.urdf_model.UrdfKinematicModel`` with batched PyTorch
4×4 transforms so FK pose-error loss backpropagates into ``Δq`` predictions.

Units: joint angles radians, tip position meters (base frame).
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from residual_adaptive_ik.kinematics.urdf_model import get_default_model

if TYPE_CHECKING:
    import torch
    from torch import nn


def _require_torch():
    try:
        import torch
        from torch import nn
    except ImportError as exc:  # pragma: no cover
        raise ImportError("PyTorch required for fk_torch") from exc
    return torch, nn


def _axis_angle_matrix_batch(
    axis: "torch.Tensor", angles: "torch.Tensor"
) -> "torch.Tensor":
    """Batched axis-angle rotation matrices, shape ``(B, 3, 3)``."""
    torch, _ = _require_torch()
    axis = axis / (torch.linalg.norm(axis) + 1e-12)
    x, y, z = axis[0], axis[1], axis[2]
    c = torch.cos(angles)
    s = torch.sin(angles)
    C = 1.0 - c
    row0 = torch.stack([c + x * x * C, x * y * C - z * s, x * z * C + y * s], dim=-1)
    row1 = torch.stack([y * x * C + z * s, c + y * y * C, y * z * C - x * s], dim=-1)
    row2 = torch.stack([z * x * C - y * s, z * y * C + x * s, c + z * z * C], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def build_fk_module():
    """Return an ``nn.Module`` mapping ``q (B,6)`` → tip position ``(B,3)``."""
    torch, nn = _require_torch()
    model = get_default_model()
    q_map = {name: i for i, name in enumerate(model.revolute_names)}

    chain_origins: list = []
    chain_axes: list = []
    chain_q_idx: list[int] = []
    chain_is_revolute: list[bool] = []
    for joint in model.chain:
        chain_origins.append(torch.tensor(joint.origin, dtype=torch.float32))
        if joint.joint_type == "revolute":
            chain_axes.append(torch.tensor(joint.axis, dtype=torch.float32))
            chain_q_idx.append(q_map[joint.name])
            chain_is_revolute.append(True)
        else:
            chain_is_revolute.append(False)

    class _BatchedFK(nn.Module):
        def forward(self, q: "torch.Tensor") -> "torch.Tensor":
            """Tip position in base frame, shape ``(B, 3)``."""
            b = q.shape[0]
            device = q.device
            dtype = q.dtype
            T = torch.eye(4, device=device, dtype=dtype).unsqueeze(0).expand(b, -1, -1)
            rev_i = 0
            for j, origin in enumerate(chain_origins):
                T = torch.bmm(
                    T, origin.to(device=device, dtype=dtype).unsqueeze(0).expand(b, -1, -1)
                )
                if chain_is_revolute[j]:
                    angles = q[:, chain_q_idx[rev_i]]
                    R = _axis_angle_matrix_batch(
                        chain_axes[rev_i].to(device), angles
                    )
                    motion = (
                        torch.eye(4, device=device, dtype=dtype)
                        .unsqueeze(0)
                        .expand(b, -1, -1)
                        .clone()
                    )
                    motion[:, :3, :3] = R
                    T = torch.bmm(T, motion)
                    rev_i += 1
            return T[:, :3, 3]

    return _BatchedFK()


def true_tip_positions_numpy(
    q_ik: np.ndarray, observed_position_error: np.ndarray
) -> np.ndarray:
    """Reconstruct true tip XYZ from IK solution + observed FK error (meters)."""
    from residual_adaptive_ik.kinematics.fk import forward_kinematics

    n = int(q_ik.shape[0])
    out = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        fk = forward_kinematics(q_ik[i])
        out[i] = (
            np.asarray(fk.position_m, dtype=np.float32)
            + np.asarray(observed_position_error[i], dtype=np.float32).reshape(3)
        )
    return out
