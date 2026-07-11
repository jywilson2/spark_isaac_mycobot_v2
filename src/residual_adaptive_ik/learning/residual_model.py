# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Supervised residual MLP (bounded Δq).

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

try:
    import torch
    from torch import nn
except ImportError:  # pragma: no cover - torch optional until Phase 2
    torch = None  # type: ignore
    nn = None  # type: ignore


if nn is not None:

    class ResidualIKModel(nn.Module):
        """MLP that predicts bounded residual joint corrections Δq (radians)."""

        def __init__(self, obs_dim: int, hidden_sizes: list[int], output_limit_rad: float) -> None:
            super().__init__()
            self.output_limit_rad = output_limit_rad
            layers: list[nn.Module] = []
            prev = obs_dim
            for h in hidden_sizes:
                layers.extend([nn.Linear(prev, h), nn.ReLU()])
                prev = h
            layers.append(nn.Linear(prev, 6))
            self.net = nn.Sequential(*layers)

        def forward(self, obs: "torch.Tensor") -> "torch.Tensor":
            """Return bounded residual joint correction Δq."""
            raw = self.net(obs)
            return torch.tanh(raw) * self.output_limit_rad

else:

    class ResidualIKModel:  # type: ignore[no-redef]
        """Placeholder when PyTorch is not installed."""

        def __init__(self, *args, **kwargs) -> None:
            raise ImportError("PyTorch required for ResidualIKModel")
