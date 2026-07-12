# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 2 cuRobo (NVIDIA, Apache-2.0) collision-free trajectory planning.

Why cuRobo
----------
Phase 1 joint lerp cannot keep links off the ground or out of obstacles.
cuRobo ``MotionGen`` produces collision-free joint trajectories on the DGX Spark
GPU. Classical DLS IK still supplies the tip goal; residual learning (Phases
3–4) still wraps ``q_ik``. See ``spec.md`` Phase 2.

CI / container: this module imports lazily. Without CUDA cuRobo, callers fall
back to NumPy path checks (no GPU requirement for unit tests).

Units: meters, radians, seconds.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from residual_adaptive_ik.geometry.collision import (
    SphereObstacle,
    check_config_collision,
    link_capsules_from_q,
)
from residual_adaptive_ik.kinematics.fk import forward_kinematics
from residual_adaptive_ik.kinematics.urdf_model import (
    DEFAULT_BASE_LINK,
    DEFAULT_EE_LINK,
    DEFAULT_REVOLUTE_JOINT_NAMES,
    UrdfKinematicModel,
    get_default_model,
    resolve_urdf_path,
)
from residual_adaptive_ik.planning.joint_path import plan_joint_lerp_checked

_REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_WORLD_YAML = _REPO_ROOT / "configs" / "planning" / "curobo_world.yaml"
DEFAULT_COLLISION_YAML = _REPO_ROOT / "configs" / "planning" / "collision.yaml"


def curobo_available() -> bool:
    """Return True when ``curobo`` imports and a CUDA device is visible."""
    try:
        import torch
        import curobo  # noqa: F401

        return bool(torch.cuda.is_available())
    except Exception:
        return False


def _ensure_warp_torch_shim() -> None:
    """Isaac Sim ships Warp ≥1.15 where ``warp.torch`` became top-level helpers.

    cuRobo 0.7.x still calls ``wp.torch.device_from_torch``. Install a thin
    compatibility module so MotionGen can initialize without downgrading Warp.
    """
    import sys
    import types

    import warp as wp

    existing = getattr(wp, "torch", None)
    if existing is not None and hasattr(existing, "device_from_torch"):
        return
    mod = types.ModuleType("warp.torch")
    for name in (
        "device_from_torch",
        "device_to_torch",
        "dtype_from_torch",
        "dtype_to_torch",
        "from_torch",
        "to_torch",
        "stream_from_torch",
        "stream_to_torch",
    ):
        if hasattr(wp, name):
            setattr(mod, name, getattr(wp, name))
    sys.modules["warp.torch"] = mod
    wp.torch = mod  # type: ignore[attr-defined]


def load_planning_config(path: Path | None = None) -> dict[str, Any]:
    cfg_path = Path(path) if path is not None else DEFAULT_COLLISION_YAML
    with cfg_path.open(encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


def load_world_config(path: Path | None = None) -> dict[str, Any]:
    """Load cuRobo world dict (cuboids / spheres). Includes ground plane."""
    cfg_path = Path(path) if path is not None else DEFAULT_WORLD_YAML
    with cfg_path.open(encoding="utf-8") as handle:
        return dict(yaml.safe_load(handle) or {})


def prepare_curobo_urdf(
    *,
    repo_root: Path | None = None,
    velocity_rad_s: float = float(np.deg2rad(160.0)),
) -> Path:
    """Return a cuRobo-ready URDF with non-zero joint velocity limits (rad/s).

    Elephant Robotics publishes ``velocity = \"0\"`` placeholders in the vendor
    URDF. cuRobo rejects zero velocity limits (``Joint velocity limits is zero``).
    Prefers the committed file under ``configs/planning/curobo/``; regenerates
    into that path when writable, otherwise into a process temp file.
    """
    import re
    import tempfile

    root = repo_root or _REPO_ROOT
    src = resolve_urdf_path()
    preferred = root / "configs" / "planning" / "curobo" / "mycobot_280_m5_curobo.urdf"
    text = src.read_text(encoding="utf-8")
    vel = f"{velocity_rad_s:.6f}"

    def _sub(match: re.Match[str]) -> str:
        return match.group(1) + vel + match.group(3)

    text2 = re.sub(
        r'(velocity\s*=\s*")([^"]*)(")',
        _sub,
        text,
        flags=re.IGNORECASE,
    )
    try:
        preferred.parent.mkdir(parents=True, exist_ok=True)
        preferred.write_text(text2, encoding="utf-8")
        return preferred
    except OSError:
        if preferred.is_file():
            return preferred
        tmp = Path(tempfile.gettempdir()) / "mycobot_280_m5_curobo.urdf"
        tmp.write_text(text2, encoding="utf-8")
        return tmp


def build_mycobot_curobo_robot_cfg(
    *,
    urdf_path: Path | str | None = None,
) -> dict[str, Any]:
    """Build a cuRobo ``robot_cfg`` dict for MyCobot 280 (coarse collision spheres).

    Spheres are intentionally conservative (tutorial). Refine later with cuRobo's
    sphere-fitting tools if mesh-accurate volumes are required.
    """
    if urdf_path is None:
        urdf = prepare_curobo_urdf()
    else:
        urdf = Path(urdf_path)
    joint_names = list(DEFAULT_REVOLUTE_JOINT_NAMES)
    # Coarse spheres in each moving link frame (meters).
    spheres = {
        "joint1": [{"center": [0.0, 0.0, 0.07], "radius": 0.035}],
        "joint2": [
            {"center": [0.0, 0.02, 0.0], "radius": 0.03},
            {"center": [0.0, 0.06, 0.0], "radius": 0.03},
        ],
        "joint3": [
            {"center": [0.0, 0.02, 0.0], "radius": 0.028},
            {"center": [0.0, 0.06, 0.0], "radius": 0.028},
        ],
        "joint4": [{"center": [0.0, 0.0, 0.0], "radius": 0.026}],
        "joint5": [{"center": [0.0, 0.0, 0.0], "radius": 0.024}],
        "joint6": [{"center": [0.0, 0.0, 0.0], "radius": 0.022}],
        "joint6_flange": [{"center": [0.0, 0.0, 0.015], "radius": 0.018}],
    }
    return {
        "robot_cfg": {
            "kinematics": {
                "usd_path": None,
                "usd_robot_root": "/robot",
                "urdf_path": str(urdf.resolve()),
                "asset_root_path": str(urdf.resolve().parent),
                "base_link": DEFAULT_BASE_LINK,
                "ee_link": DEFAULT_EE_LINK,
                "collision_link_names": [
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                    "joint6_flange",
                ],
                "collision_spheres": spheres,
                "collision_sphere_buffer": 0.0,
                "self_collision_ignore": {
                    "joint1": ["joint2"],
                    "joint2": ["joint1", "joint3"],
                    "joint3": ["joint2", "joint4"],
                    "joint4": ["joint3", "joint5"],
                    "joint5": ["joint4", "joint6", "joint6_flange"],
                    "joint6": ["joint5", "joint6_flange"],
                    "joint6_flange": ["joint5", "joint6"],
                },
                "self_collision_buffer": {
                    "joint1": 0.0,
                    "joint2": 0.0,
                    "joint3": 0.0,
                    "joint4": 0.0,
                    "joint5": 0.0,
                    "joint6": 0.0,
                    "joint6_flange": 0.0,
                },
                "use_global_cumul": True,
                "mesh_link_names": [],
                "cspace": {
                    "joint_names": joint_names,
                    "retract_config": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "null_space_weight": [1.0] * 6,
                    "cspace_distance_weight": [1.0] * 6,
                    "max_jerk": 500.0,
                    "max_acceleration": 15.0,
                },
            }
        }
    }


def check_ground_collision(
    q: np.ndarray,
    *,
    model: UrdfKinematicModel | None = None,
    link_radius_m: float = 0.025,
    ground_z_m: float = 0.0,
) -> bool:
    """True if any link capsule dips below the ground plane ``z = ground_z_m``."""
    capsules = link_capsules_from_q(q, model=model, link_radius_m=link_radius_m)
    for cap in capsules:
        z_min = float(min(cap.a_m[2], cap.b_m[2]) - cap.radius_m)
        if z_min < ground_z_m - 1e-9:
            return True
    return False


@dataclass(frozen=True)
class PlannedTrajectory:
    """Collision-aware joint trajectory.

    ``waypoints_rad`` has shape ``(T, 6)`` in radians. ``dt_s`` is the time
    between successive samples (seconds).
    """

    waypoints_rad: np.ndarray
    dt_s: float
    success: bool
    backend: str
    message: str

    @property
    def ok(self) -> bool:
        return bool(self.success) and self.waypoints_rad.size > 0


class CuRoboMotionPlanner:
    """Lazy-initialized cuRobo ``MotionGen`` wrapper for MyCobot 280."""

    def __init__(
        self,
        *,
        world_yaml: Path | None = None,
        urdf_path: Path | str | None = None,
        interpolation_dt_s: float = 0.02,
    ) -> None:
        self._world = load_world_config(world_yaml)
        self._urdf_path = urdf_path
        self._interpolation_dt_s = float(interpolation_dt_s)
        self._motion_gen = None
        self._joint_names = list(DEFAULT_REVOLUTE_JOINT_NAMES)
        self._device = None

    def _ensure(self) -> None:
        if self._motion_gen is not None:
            return
        if not curobo_available():
            raise RuntimeError("cuRobo/CUDA unavailable")
        _ensure_warp_torch_shim()
        import torch
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

        self._device = torch.device("cuda:0")
        robot_cfg = build_mycobot_curobo_robot_cfg(urdf_path=self._urdf_path)
        motion_gen_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            self._world,
            interpolation_dt=self._interpolation_dt_s,
        )
        self._motion_gen = MotionGen(motion_gen_cfg)
        self._motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)

    def plan_to_pose(
        self,
        q_start_rad: np.ndarray,
        target_position_m: np.ndarray,
        target_quaternion_wxyz: np.ndarray,
        *,
        max_attempts: int = 4,
    ) -> PlannedTrajectory:
        """Plan a collision-free trajectory to an EE pose (meters / wxyz)."""
        try:
            self._ensure()
        except Exception as exc:  # noqa: BLE001
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=self._interpolation_dt_s,
                success=False,
                backend="curobo",
                message=f"init_failed:{exc}",
            )

        import torch
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

        q0 = np.asarray(q_start_rad, dtype=np.float32).reshape(1, 6)
        pos = np.asarray(target_position_m, dtype=np.float32).reshape(3)
        quat = np.asarray(target_quaternion_wxyz, dtype=np.float32).reshape(4)
        start = JointState.from_position(
            torch.as_tensor(q0, device=self._device),
            joint_names=self._joint_names,
        )
        goal = Pose(
            position=torch.as_tensor(pos, device=self._device).view(1, 3),
            quaternion=torch.as_tensor(quat, device=self._device).view(1, 4),
        )
        assert self._motion_gen is not None
        result = self._motion_gen.plan_single(
            start, goal, MotionGenPlanConfig(max_attempts=int(max_attempts))
        )
        if not bool(result.success.item()):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=self._interpolation_dt_s,
                success=False,
                backend="curobo",
                message=f"plan_failed:{getattr(result, 'status', 'unknown')}",
            )
        traj = result.get_interpolated_plan()
        # traj.position: (T, DOF)
        waypoints = traj.position.detach().cpu().numpy().astype(float)
        dt = float(getattr(result, "interpolation_dt", self._interpolation_dt_s))
        return PlannedTrajectory(
            waypoints_rad=waypoints,
            dt_s=dt,
            success=True,
            backend="curobo",
            message="ok",
        )

    def plan_to_joint_goal(
        self,
        q_start_rad: np.ndarray,
        q_goal_rad: np.ndarray,
        *,
        model: UrdfKinematicModel | None = None,
        max_attempts: int = 4,
    ) -> PlannedTrajectory:
        """Plan to the FK pose of ``q_goal_rad`` (classical IK tip goal)."""
        mdl = model or get_default_model()
        pose = forward_kinematics(q_goal_rad, model=mdl)
        return self.plan_to_pose(
            q_start_rad,
            pose.position_m,
            pose.quaternion_wxyz,
            max_attempts=max_attempts,
        )


def plan_collision_free(
    q_start_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    *,
    prefer_curobo: bool = True,
    model: UrdfKinematicModel | None = None,
    obstacles: list[SphereObstacle] | None = None,
    planner: CuRoboMotionPlanner | None = None,
) -> PlannedTrajectory:
    """Plan a collision-free path, preferring cuRobo when available.

    Fallback: NumPy collision-checked joint lerp. If the lerp collides with
    ``obstacles`` or the ground plane, ``success=False`` (no motion).
    """
    mdl = model or get_default_model()
    cfg = load_planning_config()
    link_r = float(cfg.get("link_radius_m", 0.025))
    ground_z = float(cfg.get("ground_z_m", 0.0))
    n_samples = int(cfg.get("path_samples", 24))

    if prefer_curobo and curobo_available():
        pl = planner or CuRoboMotionPlanner()
        traj = pl.plan_to_joint_goal(q_start_rad, q_goal_rad, model=mdl)
        if traj.ok:
            return traj
        # Keep the cuRobo failure reason when falling back.
        curobo_msg = traj.message
    else:
        curobo_msg = "curobo_skipped"

    obs = list(obstacles or [])
    path = plan_joint_lerp_checked(
        q_start_rad,
        q_goal_rad,
        obs,
        n_samples=n_samples,
        model=mdl,
        link_radius_m=link_r,
        ignore_tip_segment=True,
    )
    # Ground checks along the lerp.
    ground_hit = False
    for q in path.waypoints_rad:
        if check_ground_collision(
            q, model=mdl, link_radius_m=link_r, ground_z_m=ground_z
        ):
            ground_hit = True
            break
    if path.collision.collides or ground_hit:
        reasons = list(path.collision.reasons)
        if ground_hit:
            reasons.append("ground_collision")
        return PlannedTrajectory(
            waypoints_rad=np.zeros((0, 6)),
            dt_s=1.0 / 60.0,
            success=False,
            backend="numpy_lerp",
            message=f"rejected:{','.join(reasons[:12])}|curobo={curobo_msg}",
        )
    return PlannedTrajectory(
        waypoints_rad=path.waypoints_rad,
        dt_s=1.0 / 60.0,
        success=True,
        backend="numpy_lerp",
        message=f"ok_fallback|curobo={curobo_msg}",
    )
