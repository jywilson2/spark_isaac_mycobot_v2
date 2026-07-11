# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Detect Isaac Lab / Omniverse imports when run via ``isaaclab.sh -p``.

Phase 3 prerequisite check. Not used by Phase 1 classical IK.
"""
from __future__ import annotations


def main() -> int:
    try:
        import isaaclab  # noqa: F401

        print("isaaclab: OK")
    except ImportError as exc:
        print(f"isaaclab: MISSING ({exc})")
        return 1
    try:
        import omni  # noqa: F401

        print("omni: OK")
    except ImportError as exc:
        print(f"omni: optional/missing ({exc})")
    print("detect_isaac_lab: success")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
