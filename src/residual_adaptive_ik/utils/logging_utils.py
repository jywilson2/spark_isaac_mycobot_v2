# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""CSV/JSON experiment logging helpers.

See ``spec.md`` for phase requirements and acceptance criteria.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")
