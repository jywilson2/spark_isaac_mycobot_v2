# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Shared pytest hooks for residual_adaptive_ik tests.

Why a UID-scoped basetemp
-------------------------
Container pytest often runs as root while ``USER`` / ``LOGNAME`` is the host
desktop account (e.g. ``jywilson``). Pytest then creates
``/tmp/pytest-of-jywilson`` owned by root. A later host-shell ``pytest`` as
``jywilson`` raises::

    OSError: The temporary directory /tmp/pytest-of-jywilson is not owned by
    the current user

Scoping basetemp by ``os.getuid()`` keeps root and host users from sharing
that directory. See ``README.md`` (pytest section) and ``run_verification.sh``.
"""
from __future__ import annotations

import os
from pathlib import Path


def pytest_configure(config) -> None:
    """Force ``--basetemp`` under ``/tmp/pytest-uid-<uid>/`` when unset."""
    existing = getattr(config.option, "basetemp", None)
    if existing:
        return
    root = Path(os.environ.get("PYTEST_BASETEMP_ROOT", f"/tmp/pytest-uid-{os.getuid()}"))
    try:
        root.mkdir(mode=0o700, parents=True, exist_ok=True)
        if os.stat(root).st_uid != os.getuid() and os.getuid() != 0:
            root = Path(f"/tmp/pytest-uid-{os.getuid()}-{os.getpid()}")
            root.mkdir(mode=0o700, parents=True, exist_ok=True)
        basetemp = root / "basetemp"
        config.option.basetemp = str(basetemp)
    except OSError:
        # Leave pytest defaults; caller may still hit the ownership error.
        return
