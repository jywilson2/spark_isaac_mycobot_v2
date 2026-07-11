# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Opt-in hardware test node. Requires ENABLE_MYCOBOT_HARDWARE_TESTS=1."""
from __future__ import annotations

import os
import sys


def main() -> None:
    if os.environ.get("ENABLE_MYCOBOT_HARDWARE_TESTS") != "1":
        print(
            "Refusing hardware test: set ENABLE_MYCOBOT_HARDWARE_TESTS=1 explicitly.",
            file=sys.stderr,
        )
        sys.exit(2)
    raise NotImplementedError("Hardware test node gated — implement after Phase 1–2 dry-run")


if __name__ == "__main__":
    main()
