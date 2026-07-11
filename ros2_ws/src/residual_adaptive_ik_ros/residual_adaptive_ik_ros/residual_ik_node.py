# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""ROS 2 residual IK node (dry-run first).

Modes: baseline_only | supervised_residual | sac_residual | validation_only

Hardware motion is forbidden unless ENABLE_MYCOBOT_HARDWARE_TESTS=1.
"""
from __future__ import annotations

import os


def main() -> None:
    """Entry point for ``residual_ik_node``."""
    if os.environ.get("ENABLE_MYCOBOT_HARDWARE_TESTS") == "1":
        raise RuntimeError(
            "Hardware path not implemented yet; keep ENABLE_MYCOBOT_HARDWARE_TESTS unset "
            "for dry-run skeleton."
        )
    raise NotImplementedError(
        "Implement dry-run residual IK node — see spec.md § ROS 2 Hardware Integration"
    )


if __name__ == "__main__":
    main()
