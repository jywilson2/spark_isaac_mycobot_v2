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
from residual_adaptive_ik.planning.sphere_fit_mycobot import (
    DEFAULT_TIP_LINKS_IGNORE_TARGET,
    adjacent_self_collision_ignore,
    load_collision_spheres_yaml,
)

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
    spheres_yaml: Path | None = None,
    omit_tip_links: bool = False,
) -> dict[str, Any]:
    """Build a cuRobo ``robot_cfg`` dict for MyCobot 280.

    Collision spheres come from cuRobo mesh fitting
    (``configs/planning/curobo/mycobot_280_collision_spheres.yaml``), not
    hand-tuned radii. See ``sphere_fit_mycobot.py``.

    omit_tip_links:
        When True, drop tip links (default ``joint6_flange``) from collision
        spheres so the tip may occupy the volumetric IK target at contact
        while proximal EE / arm spheres still cannot sweep through it.
    """
    if urdf_path is None:
        urdf = prepare_curobo_urdf()
    else:
        urdf = Path(urdf_path)
    joint_names = list(DEFAULT_REVOLUTE_JOINT_NAMES)
    fitted = load_collision_spheres_yaml(spheres_yaml)
    spheres = dict(fitted["collision_spheres"])
    tip_links = list(
        fitted.get("tip_links_ignore_target", DEFAULT_TIP_LINKS_IGNORE_TARGET)
    )
    if omit_tip_links:
        for tip in tip_links:
            spheres.pop(tip, None)
    collision_links = list(spheres.keys())
    ignore = fitted.get("self_collision_ignore") or adjacent_self_collision_ignore(
        tuple(collision_links)
    )
    # Keep ignore entries only for links we still collide.
    ignore = {
        k: [x for x in v if x in spheres]
        for k, v in ignore.items()
        if k in spheres
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
                "collision_link_names": collision_links,
                "collision_spheres": spheres,
                "collision_sphere_buffer": 0.0,
                "self_collision_ignore": ignore,
                "self_collision_buffer": {name: 0.0 for name in collision_links},
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


def tip_on_sphere_surface(
    tip_start_m: np.ndarray,
    sphere_center_m: np.ndarray,
    sphere_radius_m: float,
    *,
    margin_m: float = 0.008,
) -> np.ndarray:
    """Return a tip position on the near surface of a sphere obstacle (meters).

    Standoff = ``sphere_radius_m + margin_m`` along the approach from
    ``tip_start_m`` toward ``sphere_center_m``. Keeps fitted EE/flange spheres
    outside the marker volume while still allowing red→green tip contact once
    the tip reaches the surface (distance ≤ marker radius).
    """
    start = np.asarray(tip_start_m, dtype=float).reshape(3)
    center = np.asarray(sphere_center_m, dtype=float).reshape(3)
    delta = center - start
    dist = float(np.linalg.norm(delta))
    standoff = float(sphere_radius_m) + float(margin_m)
    if dist < 1e-9:
        return center - np.array([0.0, 0.0, standoff], dtype=float)
    if dist <= standoff:
        return start.copy()
    return center - (standoff / dist) * delta


class CuRoboMotionPlanner:
    """Lazy-initialized cuRobo ``MotionGen`` wrapper for MyCobot 280."""

    def __init__(
        self,
        *,
        world_yaml: Path | None = None,
        urdf_path: Path | str | None = None,
        interpolation_dt_s: float = 0.02,
        omit_tip_links: bool = False,
    ) -> None:
        self._world_base = load_world_config(world_yaml)
        self._urdf_path = urdf_path
        self._interpolation_dt_s = float(interpolation_dt_s)
        # Keep tip/flange spheres so the marker cannot pass through the EE side.
        # With a volumetric target, plan tip onto the marker surface (see
        # ``plan_to_joint_goal``) rather than into the center.
        self._omit_tip_links = bool(omit_tip_links)
        self._motion_gen = None
        self._joint_names = list(DEFAULT_REVOLUTE_JOINT_NAMES)
        self._device = None

    def _world_dict_with_obstacles(
        self, obstacles: list[SphereObstacle] | None
    ) -> dict[str, Any]:
        """Merge ground cuboids with volumetric sphere obstacles (meters).

        Important (cuRobo primitive checker):
        ``WorldConfig.sphere`` is kept on the CPU model but is **not** queried by
        the default PRIMITIVE collision checker (``collision_types={'primitive':
        True}``). Callers must run ``WorldConfig.create_obb_world`` so spheres
        become axis-aligned cuboids that the GPU checker actually uses. See
        ``scripts/host/verify_target_obstacle.sh``.
        """
        world = dict(self._world_base)
        sphere_map: dict[str, Any] = {}
        if obstacles:
            for i, obs in enumerate(obstacles):
                name = "ik_target" if i == 0 else f"obstacle_{i}"
                c = np.asarray(obs.center_m, dtype=float).reshape(3)
                sphere_map[name] = {
                    "radius": float(obs.radius_m),
                    "pose": [
                        float(c[0]),
                        float(c[1]),
                        float(c[2]),
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                    ],
                }
        world["sphere"] = sphere_map
        return world

    def _world_config_for_checker(
        self, obstacles: list[SphereObstacle] | None
    ):
        """Build a WorldConfig whose obstacles the GPU checker will query."""
        from curobo.geom.types import WorldConfig

        raw = WorldConfig.from_dict(self._world_dict_with_obstacles(obstacles))
        # Convert spheres/capsules → cuboids (OBBs) for PRIMITIVE checker.
        return WorldConfig.create_obb_world(raw)

    def _ensure(self) -> None:
        if self._motion_gen is not None:
            return
        if not curobo_available():
            raise RuntimeError("cuRobo/CUDA unavailable")
        _ensure_warp_torch_shim()
        import torch
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

        self._device = torch.device("cuda:0")
        robot_cfg = build_mycobot_curobo_robot_cfg(
            urdf_path=self._urdf_path,
            omit_tip_links=self._omit_tip_links,
        )
        # Reserve cuboid cache slots: ground + at least one IK target OBB.
        world0 = self._world_config_for_checker(None)
        motion_gen_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            world0,
            interpolation_dt=self._interpolation_dt_s,
            collision_cache={"obb": 8},
        )
        self._motion_gen = MotionGen(motion_gen_cfg)
        self._motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)

    def _apply_obstacles(self, obstacles: list[SphereObstacle] | None) -> None:
        assert self._motion_gen is not None
        self._motion_gen.update_world(self._world_config_for_checker(obstacles))

    def plan_to_pose(
        self,
        q_start_rad: np.ndarray,
        target_position_m: np.ndarray,
        target_quaternion_wxyz: np.ndarray,
        *,
        max_attempts: int = 4,
        obstacles: list[SphereObstacle] | None = None,
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

        try:
            self._apply_obstacles(obstacles)
        except Exception as exc:  # noqa: BLE001
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=self._interpolation_dt_s,
                success=False,
                backend="curobo",
                message=f"world_update_failed:{exc}",
            )

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
        obstacles: list[SphereObstacle] | None = None,
    ) -> PlannedTrajectory:
        """Plan to the FK pose of ``q_goal_rad`` (classical IK tip goal).

        When ``obstacles`` includes a volumetric marker at the tip goal, the
        MotionGen tip target is placed on the **near surface** of that sphere
        (plus a small margin) so flange collision spheres are not forced through
        the marker. Green contact still fires when the tip reaches the surface.
        """
        mdl = model or get_default_model()
        pose = forward_kinematics(q_goal_rad, model=mdl)
        tip_goal = np.asarray(pose.position_m, dtype=float).reshape(3)
        tip_start = forward_kinematics(q_start_rad, model=mdl).position_m
        tip_plan = tip_goal
        obs = list(obstacles or [])
        if obs:
            # Prefer an obstacle whose center is the tip goal (IK marker).
            marker = obs[0]
            for candidate in obs:
                if float(
                    np.linalg.norm(
                        np.asarray(candidate.center_m, dtype=float).reshape(3) - tip_goal
                    )
                ) < 1e-4:
                    marker = candidate
                    break
            tip_plan = tip_on_sphere_surface(
                tip_start, marker.center_m, float(marker.radius_m)
            )
        return self.plan_to_pose(
            q_start_rad,
            tip_plan,
            pose.quaternion_wxyz,
            max_attempts=max_attempts,
            obstacles=obstacles,
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

    ``obstacles`` are volumetric spheres (meters). The IK target marker must be
    passed as a ``SphereObstacle``. Internally, spheres are converted to OBBs
    via ``WorldConfig.create_obb_world`` so cuRobo's PRIMITIVE checker queries
    them (raw ``WorldConfig.sphere`` alone is **not** enough).

    Policy (fail closed on host)
    ---------------------------
    When cuRobo is preferred **and** available, a cuRobo reject is final:
    we do **not** execute a NumPy joint lerp afterward. Coarse capsules often
    miss EE-side vs marker contact, which produced GUI motion that still
    clipped the target after logs showed ``plan_failed`` inside an
    ``ok_fallback`` message.

    NumPy collision-checked lerp is used only when cuRobo is unavailable or
    ``prefer_curobo=False`` (CI). Optional escape hatch:
    ``fallback_numpy_after_curobo_fail: true`` in ``collision.yaml``.
    """
    mdl = model or get_default_model()
    cfg = load_planning_config()
    link_r = float(cfg.get("link_radius_m", 0.025))
    ground_z = float(cfg.get("ground_z_m", 0.0))
    n_samples = int(cfg.get("path_samples", 24))
    max_attempts = int(cfg.get("curobo_max_attempts", 4))
    obs = list(obstacles or [])

    if prefer_curobo and curobo_available():
        pl = planner or CuRoboMotionPlanner(omit_tip_links=False)
        traj = pl.plan_to_joint_goal(
            q_start_rad,
            q_goal_rad,
            model=mdl,
            obstacles=obs,
            max_attempts=max_attempts,
        )
        if traj.ok:
            return traj
        # Fail closed unless explicitly allowed (unsafe for volumetric marker).
        if not bool(cfg.get("fallback_numpy_after_curobo_fail", False)):
            return PlannedTrajectory(
                waypoints_rad=np.zeros((0, 6)),
                dt_s=1.0 / 60.0,
                success=False,
                backend="curobo",
                message=f"rejected_no_numpy_fallback|{traj.message}",
            )
        curobo_msg = traj.message
    else:
        curobo_msg = "curobo_skipped"

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
