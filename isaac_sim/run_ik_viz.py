#!/usr/bin/env python3
# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 1 classical IK metrics **with** Isaac Sim visualization (host).

Also runs Phase 2 collision-aware planning (cuRobo / recovery) during animation.

One run:
  1. Evaluates ``--num-poses`` DLS IK trials (Phase 1 metrics).
  2. Writes JSON + Markdown reports.
  3. Animates a representative subset. The goal sphere relocates only after
     planning finishes: **red** then EE motion on PLAN_OK; **yellow** (no
     motion) after recovery timeout/exhaustion. See ``isaac_sim/target_marker.py``.

Run on the **host**:

    ./scripts/host/run_isaac_viz.sh
    # quick test:
    ./scripts/host/run_isaac_viz.sh --skip-tests -- \\
        --num-poses 240 --visualize 48 --hold-s 0.4

See ``spec.md`` Phases 1–2. Legacy name: ``run_ik_viz.py``.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path
from typing import Callable

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
_SRC = REPO_ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

# #region agent log
_DEBUG_LOG_PATH = REPO_ROOT / ".cursor" / "debug-4285da.log"


def _agent_debug_log(
    hypothesis_id: str,
    location: str,
    message: str,
    data: dict,
    *,
    run_id: str = "graze-debug",
) -> None:
    """Append one NDJSON debug line (host-safe via REPO_ROOT)."""
    try:
        import json as _json

        _DEBUG_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(_DEBUG_LOG_PATH, "a", encoding="utf-8") as _df:
            _df.write(
                _json.dumps(
                    {
                        "sessionId": "4285da",
                        "runId": run_id,
                        "hypothesisId": hypothesis_id,
                        "location": location,
                        "message": message,
                        "data": data,
                        "timestamp": int(time.time() * 1000),
                    }
                )
                + "\n"
            )
    except Exception:
        pass


# #endregion

from isaac_sim.target_marker import (  # noqa: E402
    TARGET_MARKER_BASE_KEEPOUT_XY_M,
    TARGET_MARKER_BASE_KEEPOUT_Z_M,
    TARGET_MARKER_CONTACT_DISTANCE_M,
    TARGET_MARKER_RADIUS_M,
    TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
    TARGET_MARKER_TOOL_AXIS_TOL_RAD,
    classify_tip_contact,
    ee_contacts_target,
    marker_rgb_for_state,
)
from isaac_sim.viz_plan_policy import (  # noqa: E402
    MarkerVisualState,
    ViaPressureTracker,
    may_execute_motion,
    meets_max_skip_unreachable_frac,
    meets_min_plan_ok_rate,
    plan_ok_rate,
    plan_result_is_executable,
    skip_unreachable_frac,
)
from isaac_sim.urdf_import import (  # noqa: E402
    URDF_IMPORTER_EXTENSION,
    add_robot_reference_to_stage,
    import_urdf_to_usd,
)
from isaac_sim.urdf_utils import (  # noqa: E402
    REVOLUTE_JOINT_NAMES,
    ROBOT_PRIM_PATH,
    default_prepared_urdf,
    prepare_robot_assets,
)
from residual_adaptive_ik.kinematics.baseline_eval import (  # noqa: E402
    evaluate_baseline,
    metrics_to_markdown,
    select_trials_for_visualization,
)
from residual_adaptive_ik.kinematics.fk import forward_kinematics  # noqa: E402
from residual_adaptive_ik.kinematics.robot_home import (  # noqa: E402
    load_home_joint_positions_rad,
)
from residual_adaptive_ik.utils.logging_utils import write_json  # noqa: E402
from residual_adaptive_ik.kinematics.workspace_sampling import (  # noqa: E402
    load_workspace_config,
)
from residual_adaptive_ik.geometry import SphereObstacle  # noqa: E402
from residual_adaptive_ik.geometry.collision import (  # noqa: E402
    capsule_sphere_collide,
    link_capsules_from_q,
    proximal_arm_contacts_target,
)
from residual_adaptive_ik.planning import (  # noqa: E402
    curobo_available,
)
from residual_adaptive_ik.planning.curobo_planner import (  # noqa: E402
    CuRoboMotionPlanner,
    load_planning_config,
)
from residual_adaptive_ik.planning.recovery import (  # noqa: E402
    plan_collision_free_with_recovery,
    recovery_via_attempts_in_message,
)
from residual_adaptive_ik.planning.dexterity import (  # noqa: E402
    contact_pose_is_dexterous,
    is_in_dexterous_region,
)
from residual_adaptive_ik.planning.skipped_analysis import (  # noqa: E402
    analyze_and_report,
)

# A recovery that needs at least this many standoff vias while the EE started
# near the target is flagged HIGH_RETRY_WHEN_CLOSE for diagnosis (see EE_CLOSE).
HIGH_RETRY_VIA_THRESHOLD = 5


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 1 DLS IK metrics with Isaac Sim rendering."
    )
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable GUI (still steps the sim; useful for smoke tests)",
    )
    parser.add_argument(
        "--num-poses",
        type=int,
        default=1000,
        help="Number of reachable poses to evaluate for metrics (default: 1000)",
    )
    parser.add_argument(
        "--visualize",
        type=int,
        default=48,
        help="How many of those trials to animate in Isaac Sim (default: 48)",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=0.75,
        help="Seconds to hold each visualized IK solution (wall-clock; scaled by --time-warp)",
    )
    parser.add_argument(
        "--time-warp",
        type=float,
        default=1.0,
        help="Playback speed multiplier for joint motion + holds (1.0 = real-time hardware "
        "cap). Headless smokes typically use >1 (no human watching). Does not change "
        "planner physics — only articulation playback / hold waits.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="RNG seed for reachable pose sampling",
    )
    parser.add_argument(
        "--keep-prepared",
        action="store_true",
        help="Reuse existing assets/robots/mycobot_280_m5 if present",
    )
    parser.add_argument(
        "--save-usd",
        type=Path,
        default=None,
        help="Scene USD export path (default: assets/usd/phase1_mycobot.usd)",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Metrics JSON path (default: assets/logs/phase1_baseline_metrics.json)",
    )
    parser.add_argument(
        "--md-out",
        type=Path,
        default=None,
        help="Metrics markdown path (default: docs/phase1_baseline.md)",
    )
    parser.add_argument(
        "--import-only",
        action="store_true",
        help="Import URDF→USD and exit (no metrics / animation)",
    )
    parser.add_argument(
        "--metrics-only",
        action="store_true",
        help="Compute and write metrics inside Kit process, skip articulation animation",
    )
    parser.add_argument(
        "--auto-exit",
        action="store_true",
        help="After GUI visualization, close Kit instead of waiting for the window to be closed "
        "(required for agent / nsenter GUI smoke without manual intervention)",
    )
    parser.add_argument(
        "--no-workspace-filter",
        action="store_true",
        help="Do not filter samples by workspace radius",
    )
    home = parser.add_mutually_exclusive_group()
    home.add_argument(
        "--reset-to-home",
        dest="reset_to_home",
        action="store_true",
        help="Independent-episode mode: return to home_joint_positions_rad "
        "before each viz trial (opt-in 1.0 rate-gate benchmark)",
    )
    home.add_argument(
        "--no-reset-to-home",
        dest="reset_to_home",
        action="store_false",
        help="Sequential multi-target (default): home once at session start; "
        "do not reset between trials (path-dependent recovery)",
    )
    parser.set_defaults(reset_to_home=False)
    parser.add_argument(
        "--min-plan-ok-rate",
        type=float,
        default=None,
        help="Fail viz if PLAN_OK/(OK+FAIL) is below this (default: "
        "configs/planning/collision.yaml min_plan_ok_rate or "
        "ISAAC_VIZ_MIN_PLAN_OK_RATE). Use 0 to disable.",
    )
    parser.add_argument(
        "--max-skip-unreachable-frac",
        type=float,
        default=None,
        help="Fail viz if SKIPPED_UNREACHABLE/candidates exceeds this "
        "(default: collision.yaml max_skip_unreachable_frac or "
        "ISAAC_VIZ_MAX_SKIP_UNREACHABLE_FRAC). Use >=1 to disable.",
    )
    spheres = parser.add_mutually_exclusive_group()
    spheres.add_argument(
        "--show-collision-spheres",
        dest="show_collision_spheres",
        action="store_true",
        help="Draw translucent mesh-fitted collision spheres (amber=arm, "
        "cyan=tip-omit links). Default off; GUI smoke enables this.",
    )
    spheres.add_argument(
        "--no-show-collision-spheres",
        dest="show_collision_spheres",
        action="store_false",
        help="Disable collision-sphere overlay (override GUI smoke default).",
    )
    parser.set_defaults(show_collision_spheres=None)
    parser.add_argument(
        "--collision-sphere-opacity",
        type=float,
        default=0.35,
        help="Opacity of collision-sphere overlay (0–1, default 0.35).",
    )
    parser.add_argument(
        "--early-abort-after-fails",
        type=int,
        default=3,
        help="Stop the viz trial loop after this many PLAN_FAIL episodes "
        "(default 3), regardless of PLAN_OK count. Use 1 for fail-fast "
        "iteration. Set 0 to disable.",
    )
    return parser.parse_args()


def _viz_log(message: str, *, level: str = "info") -> None:
    """Print to host stdout and mirror into Kit Console when ``carb`` is loaded.

    Host terminal output does not automatically appear in the Isaac Sim UI.
    ``carb.log_*`` / Python ``logging`` show under **Window → Console** (set
    filter to Info/Verbose for info-level lines).
    """
    print(message)
    try:
        import carb  # noqa: WPS433

        if level == "warn":
            carb.log_warn(message)
        elif level == "error":
            carb.log_error(message)
        else:
            carb.log_info(message)
    except Exception:
        pass


def _set_target_marker_color(stage, *, state: MarkerVisualState) -> None:
    """Update existing IkTarget material color (red / green / yellow).

    Color is applied only via UsdPreviewSurface — do **not** set
    ``primvars:displayColor`` without indices (Fabric warns otherwise).
    """
    from pxr import Gf, Sdf, UsdShade  # noqa: WPS433

    color_rgb, emit_rgb = marker_rgb_for_state(state)
    color = Gf.Vec3f(*color_rgb)
    emit = Gf.Vec3f(*emit_rgb)
    shader_path = Sdf.Path("/World/IkTargetMaterial/Shader")
    shader_prim = stage.GetPrimAtPath(shader_path)
    if not shader_prim.IsValid():
        return
    shader = UsdShade.Shader(shader_prim)
    diff = shader.GetInput("diffuseColor")
    emis = shader.GetInput("emissiveColor")
    if diff:
        diff.Set(color)
    if emis:
        emis.Set(emit)


def _set_target_marker(
    stage,
    position_m: np.ndarray,
    *,
    state: MarkerVisualState = MarkerVisualState.PENDING,
    radius_m: float = TARGET_MARKER_RADIUS_M,
) -> None:
    """Place a small sphere at the IK target (meters).

    Starts **red** (pending). Turns **green** on tip contact after a successful
    plan, or **yellow** when path planning fails (no motion).

    Phase 2 treats this marker as a **volumetric** sphere obstacle (same radius)
    for cuRobo / NumPy planning — fitted EE and arm volumes must not intersect
    it on the path.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: WPS433

    path = Sdf.Path("/World/IkTarget")
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.GetRadiusAttr().Set(float(radius_m))
    color_rgb, emit_rgb = marker_rgb_for_state(state)
    # Material only — avoid CreateDisplayColorAttr (triggers Fabric
    # primvars:displayColor:indices warnings without an indices primvar).
    xform = UsdGeom.Xformable(sphere)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(float(position_m[0]), float(position_m[1]), float(position_m[2]))
    )
    mat_path = Sdf.Path("/World/IkTargetMaterial")
    if stage.GetPrimAtPath(mat_path).IsValid():
        stage.RemovePrim(mat_path)
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendPath("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color_rgb))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit_rgb))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(sphere).Bind(mat)



def _create_articulation(prim_path: str):
    try:
        from isaacsim.core.prims import SingleArticulation  # noqa: WPS433

        art = SingleArticulation(prim_path=prim_path)
        art.initialize()
        return art
    except Exception:
        pass
    try:
        from omni.isaac.core.articulations import Articulation  # noqa: WPS433

        art = Articulation(prim_path=prim_path)
        art.initialize()
        return art
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            f"Could not create Articulation at {prim_path}. "
            "Check Isaac Sim version / robot USD import."
        ) from exc


def _set_joint_positions(articulation, q_rad: np.ndarray) -> None:
    q = np.asarray(q_rad, dtype=float).reshape(-1)
    names = list(REVOLUTE_JOINT_NAMES)
    if hasattr(articulation, "set_joint_positions"):
        try:
            articulation.set_joint_positions(q, joint_names=names)
            return
        except TypeError:
            articulation.set_joint_positions(q)
            return
    if hasattr(articulation, "set_joint_position_targets"):
        try:
            articulation.set_joint_position_targets(q, joint_names=names)
            return
        except TypeError:
            articulation.set_joint_position_targets(q)
            return
    raise RuntimeError("Articulation has no set_joint_positions / set_joint_position_targets")


def _get_joint_positions(articulation) -> np.ndarray:
    names = list(REVOLUTE_JOINT_NAMES)
    if hasattr(articulation, "get_joint_positions"):
        try:
            q = articulation.get_joint_positions(joint_names=names)
            return np.asarray(q, dtype=float).reshape(-1)
        except TypeError:
            q = articulation.get_joint_positions()
            return np.asarray(q, dtype=float).reshape(-1)
    if hasattr(articulation, "get_joint_position"):
        return np.asarray(
            [articulation.get_joint_position(n) for n in names], dtype=float
        )
    raise RuntimeError("Articulation has no get_joint_positions")


def _sequential_retract_tip_from_marker(
    articulation,
    simulation_app,
    target_xyz: np.ndarray,
    *,
    retract_m: float = 0.05,
    max_speed_rad_s: float,
) -> bool:
    """Pull the tip outward along the surface normal after a contact episode.

    Why
    ---
    Sequential multi-target smoke leaves the wrist on the previous marker with
    a pad-facing (or near-pad) orientation for *that* sphere. The next target's
    MotionGen approach then samples the tip on the new surface with a
    flipped/back axis (``axis_out≈π``) → ``CONTACT_INVALID_MIDPATH_GRAZE``.
    A short outward retract clears the shell before the next plan.
    """
    try:
        from residual_adaptive_ik.kinematics.fk import Pose
        from residual_adaptive_ik.kinematics.numerical_ik import DampedLeastSquaresIK

        q_now = _get_joint_positions(articulation)
        pose = forward_kinematics(q_now)
        tip = np.asarray(pose.position_m, dtype=float).reshape(3)
        tgt = np.asarray(target_xyz, dtype=float).reshape(3)
        v = tip - tgt
        n = float(np.linalg.norm(v))
        if n < 1e-6:
            return False
        tip_goal = tip + (v / n) * max(0.01, float(retract_m))
        ik = DampedLeastSquaresIK(
            max_iterations=80,
            damping=2e-3,
            position_tol_m=2e-3,
            orientation_tol_rad=0.08,
        )
        res = ik.solve(
            Pose(position_m=tip_goal, quaternion_wxyz=pose.quaternion_wxyz),
            seed_q=q_now,
        )
        if not bool(getattr(res, "success", False)):
            _viz_log(
                f"  SEQUENTIAL_RETRACT: IK fail ({getattr(res, 'reason', '?')})",
                level="warn",
            )
            return False
        _move_joints_at_hardware_speed(
            articulation,
            res.q,
            simulation_app,
            max_speed_rad_s=max_speed_rad_s,
        )
        tip_after = np.asarray(
            forward_kinematics(
                _get_joint_positions(articulation)
            ).position_m,
            dtype=float,
        ).reshape(3)
        _viz_log(
            "  SEQUENTIAL_RETRACT: tip cleared marker "
            f"(dist={float(np.linalg.norm(tip_after - tgt)) * 1e3:.1f}mm, "
            f"retract_m={float(retract_m):.3f})"
        )
        return True
    except Exception as exc:  # noqa: BLE001
        _viz_log(f"  SEQUENTIAL_RETRACT: skipped ({exc})", level="warn")
        return False


def _move_joints_at_hardware_speed(
    articulation,
    q_goal_rad: np.ndarray,
    simulation_app,
    *,
    max_speed_rad_s: float,
    dt_s: float = 1.0 / 60.0,
    on_step: Callable[[np.ndarray], None] | None = None,
) -> None:
    """Interpolate joint motion capped at vendor max joint speed (rad/s).

    Matches Elephant Robotics myCobot 280 ``Joint Maximum Speed`` (160 °/s).
    Optional ``on_step(q_rad)`` runs after each joint update (e.g. contact color).
    """
    q_goal = np.asarray(q_goal_rad, dtype=float).reshape(-1)
    try:
        q = _get_joint_positions(articulation)
    except Exception:
        q = q_goal.copy()
        _set_joint_positions(articulation, q)
        if on_step is not None:
            on_step(q)
        return
    if q.shape != q_goal.shape:
        q = q_goal.copy()
        _set_joint_positions(articulation, q)
        if on_step is not None:
            on_step(q)
        return
    speed = max(1e-6, float(max_speed_rad_s))
    step_limit = speed * max(1e-4, float(dt_s))
    while simulation_app.is_running():
        err = q_goal - q
        max_err = float(np.max(np.abs(err)))
        if max_err < 1e-4:
            break
        scale = min(1.0, step_limit / max_err)
        q = q + err * scale
        _set_joint_positions(articulation, q)
        if on_step is not None:
            on_step(q)
        simulation_app.update()
    _set_joint_positions(articulation, q_goal)
    if on_step is not None:
        on_step(q_goal)


def _follow_trajectory(
    articulation,
    waypoints_rad: np.ndarray,
    simulation_app,
    *,
    dt_s: float,
    max_speed_rad_s: float,
    on_step: Callable[[np.ndarray], None] | None = None,
) -> None:
    """Play back a joint trajectory, still respecting vendor max joint speed.

    If consecutive waypoints imply speeds above ``max_speed_rad_s``, intermediate
    lerp samples are inserted (radians / seconds).
    """
    wps = np.asarray(waypoints_rad, dtype=float)
    if wps.ndim != 2 or wps.shape[0] == 0:
        return
    speed = max(1e-6, float(max_speed_rad_s))
    dt = max(1e-4, float(dt_s))
    for i in range(wps.shape[0]):
        if not simulation_app.is_running():
            break
        q_goal = wps[i]
        try:
            q = _get_joint_positions(articulation)
        except Exception:
            q = q_goal.copy()
        while simulation_app.is_running():
            err = q_goal - q
            max_err = float(np.max(np.abs(err)))
            if max_err < 1e-4:
                break
            scale = min(1.0, (speed * dt) / max_err)
            q = q + err * scale
            _set_joint_positions(articulation, q)
            if on_step is not None:
                on_step(q)
            simulation_app.update()
        _set_joint_positions(articulation, q_goal)
        if on_step is not None:
            on_step(q_goal)
        simulation_app.update()


def _print_metrics_summary(metrics: dict) -> None:
    print(
        f"Phase 1 metrics: success_rate={metrics['success_rate']:.4f} "
        f"({metrics['n_success']}/{metrics['n_poses']}) "
        f"median_pos_err_m={metrics['median_position_error_m']:.6e} "
        f"p95_pos_err_m={metrics['p95_position_error_m']:.6e}"
    )


def run_viz(args: argparse.Namespace) -> int:
    from isaacsim import SimulationApp  # noqa: WPS433

    repo_root = args.repo_root.resolve()
    json_out = args.json_out or (repo_root / "assets" / "logs" / "phase1_baseline_metrics.json")
    md_out = args.md_out or (repo_root / "docs" / "phase1_baseline.md")
    prepared = default_prepared_urdf(repo_root)
    if not (args.keep_prepared and prepared.is_file()):
        prepared = prepare_robot_assets(repo_root)
    robot_usd = prepared.with_suffix(".usd")
    save_usd = args.save_usd or (repo_root / "assets" / "usd" / "phase1_mycobot.usd")

    simulation_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "extra_args": ["--enable", URDF_IMPORTER_EXTENSION],
        }
    )

    try:
        import omni.timeline  # noqa: WPS433
        import omni.usd  # noqa: WPS433
        from isaacsim.core.utils.stage import create_new_stage  # noqa: WPS433
        from pxr import Gf, Sdf, UsdGeom, UsdLux  # noqa: WPS433

        print(f"Prepared URDF: {prepared}")
        robot_usd_path = import_urdf_to_usd(
            prepared_urdf=prepared,
            output_usd=robot_usd,
            simulation_app=simulation_app,
        )
        print(f"Robot USD: {robot_usd_path}")

        create_new_stage()
        prim_path = add_robot_reference_to_stage(robot_usd_path, ROBOT_PRIM_PATH)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        world_path = Sdf.Path("/World")
        if not stage.GetPrimAtPath(world_path).IsValid():
            UsdGeom.Xform.Define(stage, world_path)

        ground = UsdGeom.Cube.Define(stage, world_path.AppendPath("GroundPlane"))
        ground.GetSizeAttr().Set(10.0)
        gxf = UsdGeom.Xformable(ground)
        # Top surface ≈ z=-0.02 m (matches configs/planning/curobo_world.yaml).
        gxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.07))
        gxf.AddScaleOp().Set(Gf.Vec3d(10.0, 10.0, 0.01))

        dome = UsdLux.DomeLight.Define(stage, world_path.AppendPath("DomeLight"))
        dome.CreateIntensityAttr().Set(1000.0)

        save_usd = save_usd.resolve()
        save_usd.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(save_usd))
        print(f"Saved scene USD: {save_usd}")

        if args.import_only:
            print("--import-only: skipping metrics and animation")
            return 0

        print(f"Evaluating Phase 1 metrics on {args.num_poses} poses (seed={args.seed})...")
        workspace = load_workspace_config()
        metrics, trials = evaluate_baseline(
            n_poses=int(args.num_poses),
            seed=int(args.seed),
            filter_workspace=not args.no_workspace_filter,
            return_trials=True,
            sampling="stratified",
        )
        # Candidate pool for Phase 2 motion: all IK successes (spatially
        # spread when possible). ``--visualize N`` means N *countable*
        # episodes (PLAN_OK / PLAN_FAIL). SKIPPED_UNREACHABLE / overlapping
        # targets do **not** consume an episode slot (spec.md).
        n_success = sum(1 for t in trials if t.success)
        candidate_cap = max(int(args.visualize) * 4, n_success, int(args.visualize))
        to_show = select_trials_for_visualization(
            trials, max_visualize=min(candidate_cap, max(n_success, int(args.visualize)))
        )
        if n_success > len(to_show):
            # Ensure the full success pool is available so skips can be
            # replaced by later candidates until N countable episodes fill.
            seen = {t.index for t in to_show}
            for t in trials:
                if t.success and t.index not in seen:
                    to_show.append(t)
                    seen.add(t.index)
        metrics["isaac_viz"] = True
        metrics["n_visualized_target"] = int(args.visualize)
        metrics["n_candidate_pool"] = len(to_show)
        metrics["generated_by"] = (
            "`isaac_sim/run_ik_viz.py` / `scripts/host/run_isaac_viz.sh`"
        )
        write_json(json_out, metrics)
        md_out.parent.mkdir(parents=True, exist_ok=True)
        md_out.write_text(metrics_to_markdown(metrics))
        print(f"Wrote {json_out}")
        print(f"Wrote {md_out}")
        _print_metrics_summary(metrics)

        if args.metrics_only or int(args.visualize) <= 0:
            print("Skipping articulation animation (--metrics-only or --visualize 0).")
            return 0

        time_warp = max(1e-3, float(getattr(args, "time_warp", 1.0) or 1.0))
        hold_s = max(0.02, float(args.hold_s) / time_warp)
        print(
            f"Filling {int(args.visualize)} countable episodes from "
            f"{len(to_show)} candidates "
            f"(hold={hold_s:.3f}s wall, time_warp={time_warp:g}×; "
            f"sphere red→green on EE tip-face contact "
            f"≤{TARGET_MARKER_CONTACT_DISTANCE_M * 1e3:.0f} mm; "
            f"servo≤{workspace.max_joint_speed_deg_s * time_warp:g}°/s)..."
        )
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(10):
            simulation_app.update()

        articulation = _create_articulation(prim_path)
        # Time-warp accelerates playback for headless (no human watching). Cap
        # stays at vendor max_joint_speed * warp; hold waits shrink by the same
        # factor. Planning / contact geometry are unchanged.
        max_speed = float(workspace.max_joint_speed_rad_s) * time_warp
        if time_warp != 1.0:
            _viz_log(
                f"Phase 2 time-warp={time_warp:g}× "
                f"(joint_speed→{max_speed:.2f} rad/s, hold_s→{hold_s:.3f}s)"
            )
        plan_cfg = load_planning_config()
        gate = bool(plan_cfg.get("gate_motion_on_plan_failure", True))
        prefer_curobo = bool(plan_cfg.get("prefer_curobo", True))
        curobo_planner = None
        curobo_contact_planner = None
        if prefer_curobo and curobo_available():
            try:
                dt = float(plan_cfg.get("curobo_interpolation_dt_s", 0.02))
                curobo_planner = CuRoboMotionPlanner(interpolation_dt_s=dt)
                # Separate MotionGen without tip/flange spheres for surface contact.
                curobo_contact_planner = CuRoboMotionPlanner(
                    interpolation_dt_s=dt, omit_tip_links=True
                )
                print(
                    "Phase 2 planner: cuRobo MotionGen (CUDA) ready "
                    "(+ contact planner omit_tip_links)"
                )
            except Exception as exc:  # noqa: BLE001
                print(f"Phase 2 planner: cuRobo init failed ({exc}); NumPy fallback")
                curobo_planner = None
                curobo_contact_planner = None
        else:
            print("Phase 2 planner: NumPy collision-checked lerp (cuRobo unavailable)")

        n_plan_ok = 0
        n_plan_fail = 0
        n_contact_green = 0
        n_via_ok = 0  # targets unreachable directly; needed standoff waypoint(s)
        n_skip = 0  # overlapping targets excluded from the rate gate
        n_skip_unreachable = 0  # dexterity prescreen: contact pose IK-infeasible
        n_invalid_side = 0  # green flashed but settled contact was wrong-side
        # Structured SKIPPED_UNREACHABLE RESULT lines for the end-of-test
        # analyzer (spec.md Phase 2 SKIPPED_UNREACHABLE analysis requirement).
        skipped_unreachable_log: list[str] = []
        # Repeated-via diagnostic across consecutive episodes (math: EMA of via
        # usage + consecutive-via streak). High ⇒ systematic bad approach angle.
        via_pressure = ViaPressureTracker()
        # Dexterity / dexterous-workspace gate (Pinocchio preferred). Provably
        # infeasible pad-facing contact poses → SKIPPED_UNREACHABLE (excluded
        # from the gate), never faked as OK.
        prescreen_enabled = bool(
            plan_cfg.get("plan_prescreen_dexterity_enabled", True)
        )
        prescreen_backend = str(plan_cfg.get("plan_prescreen_backend", "auto"))
        prescreen_pos_tol = float(plan_cfg.get("plan_prescreen_position_tol_m", 0.006))
        prescreen_ori_tol = float(
            plan_cfg.get("plan_prescreen_orientation_tol_rad", 0.35)
        )
        prescreen_seeds = int(plan_cfg.get("plan_prescreen_seeds", 12))
        # null → auto (True for Pinocchio, False for NumPy) inside contact_pose_is_dexterous.
        _skip_raw = plan_cfg.get("plan_prescreen_skip_orientation_infeasible", None)
        if _skip_raw is None or (
            isinstance(_skip_raw, str) and _skip_raw.strip().lower() in ("", "null", "none", "auto")
        ):
            prescreen_skip_orientation = None
        else:
            prescreen_skip_orientation = bool(_skip_raw)
        region_margin_m = float(plan_cfg.get("dexterous_region_margin_m", 0.02))
        cone_max_rad = float(plan_cfg.get("contact_orientation_cone_max_rad", 0.30))
        cone_tilts = int(plan_cfg.get("contact_orientation_cone_tilts", 2))
        cone_azimuths = int(plan_cfg.get("contact_orientation_cone_azimuths", 4))
        if prescreen_enabled:
            from residual_adaptive_ik.planning.dexterity import (
                resolve_prescreen_backend,
            )

            _be = resolve_prescreen_backend(prescreen_backend)
            _viz_log(
                f"Phase 2 dexterity gate: backend={_be} "
                f"skip_orientation={prescreen_skip_orientation} "
                f"region_margin_m={region_margin_m:.3f}"
            )

        prefer_dex = bool(plan_cfg.get("prefer_dexterous_region_candidates", True))
        if prefer_dex and to_show:
            try:
                from residual_adaptive_ik.planning.dexterity import (
                    load_reach_envelope_m,
                )

                _min_r, _max_r = load_reach_envelope_m()
            except Exception:
                _min_r, _max_r = 0.12, 0.28

            def _in_region(trial) -> bool:
                ok, _ = is_in_dexterous_region(
                    trial.target.position_m,
                    min_reach_m=float(_min_r),
                    max_reach_m=float(_max_r),
                    margin_m=region_margin_m,
                )
                return bool(ok)

            to_show = sorted(
                to_show,
                key=lambda t: (0 if _in_region(t) else 1, int(t.index)),
            )
            _viz_log(
                "Phase 2 candidate order: prefer Dexterous Region first "
                f"(margin_m={region_margin_m:.3f})"
            )

        n_countable_target = max(0, int(args.visualize))
        n_countable = 0  # PLAN_OK + PLAN_FAIL only (skips do not count)
        n_candidates_considered = 0

        def _tally() -> str:
            """One-line running status for real-time log monitoring.

            Grep-friendly: ``STATUS ok=.. fail=.. via=.. green=.. skip=..
            rate=..`` after every episode, so a tail of the log always shows
            the current totals without waiting for the end-of-run summary.
            """
            rate_now = plan_ok_rate(n_plan_ok, n_plan_fail)
            return (
                f"STATUS ok={n_plan_ok} fail={n_plan_fail} via={n_via_ok} "
                f"green={n_contact_green} skip={n_skip} "
                f"episodes={n_countable}/{n_countable_target} "
                f"rate={rate_now:.3f}"
            )
        q_home = load_home_joint_positions_rad()
        # CLI / smoke default is sequential: --no-reset-to-home (False).
        # Pass --reset-to-home only for the independent-episode 1.0 gate.
        reset_home = bool(args.reset_to_home)

        # Transparent mesh-fitted collision spheres (same YAML cuRobo uses).
        # Default: ON for GUI, OFF for headless. Override with CLI or
        # ISAAC_VIZ_SHOW_COLLISION_SPHERES=0|1.
        show_spheres = getattr(args, "show_collision_spheres", None)
        if show_spheres is None:
            env_sph = os.environ.get("ISAAC_VIZ_SHOW_COLLISION_SPHERES", "").strip().lower()
            if env_sph in ("1", "true", "yes", "on"):
                show_spheres = True
            elif env_sph in ("0", "false", "no", "off"):
                show_spheres = False
            else:
                show_spheres = not bool(args.headless)
        collision_viz = None
        if show_spheres:
            from isaac_sim.collision_sphere_viz import CollisionSphereVisualizer

            collision_viz = CollisionSphereVisualizer(
                stage,
                opacity=float(getattr(args, "collision_sphere_opacity", 0.35)),
            )
            try:
                collision_viz.update(q_home)
            except Exception as exc:  # noqa: BLE001
                _viz_log(
                    f"Collision-sphere overlay init failed ({exc}); continuing without",
                    level="warn",
                )
                collision_viz = None
            else:
                _viz_log(
                    "Collision-sphere overlay ON "
                    f"(opacity={float(getattr(args, 'collision_sphere_opacity', 0.35)):.2f}; "
                    "amber=proximal arm, cyan=tip-omit links)"
                )

        def _update_collision_spheres(q_rad: np.ndarray) -> None:
            if collision_viz is None:
                return
            try:
                collision_viz.update(np.asarray(q_rad, dtype=float).reshape(-1))
            except Exception:
                pass

        # Always move to home once before the trial loop (first episode).
        # Per-trial home is opt-in only — required Spark GUI smoke must not
        # reset between episodes (spec.md § Sequential multi-target).
        _viz_log(
            f"Phase 2: moving to home once before trials "
            f"q_home_rad={np.array2string(q_home, precision=3)}"
        )
        _move_joints_at_hardware_speed(
            articulation,
            q_home,
            simulation_app,
            max_speed_rad_s=max_speed,
            on_step=_update_collision_spheres,
        )
        if reset_home:
            _viz_log(
                "Phase 2: ALSO reset to home before each trial "
                "(opt-in --reset-to-home / independent-episode gate)"
            )
        else:
            _viz_log(
                "Phase 2: no per-trial home reset "
                "(default sequential multi-target / GUI smoke policy)"
            )
        for trial in to_show:
            if n_countable >= n_countable_target:
                break
            n_candidates_considered += 1
            status = "OK" if trial.success else f"FAIL({trial.reason})"
            # Provisional label until we know if this candidate becomes a
            # countable episode (skips keep the candidate index).
            cand = f"[cand {n_candidates_considered}]"
            _viz_log(
                f"{cand} CANDIDATE trial={trial.index} ik={status} "
                f"pos_err_m={trial.position_error_m:.4e} "
                f"target=({trial.target.position_m[0]:.3f},"
                f"{trial.target.position_m[1]:.3f},"
                f"{trial.target.position_m[2]:.3f})"
            )
            if not trial.success:
                _viz_log(
                    f"{cand} RESULT IK_FAIL reason={trial.reason} | {_tally()}",
                    level="warn",
                )
                continue
            # Episode label assigned only after skip filters (below).
            episode = ""

            target_xyz = np.asarray(trial.target.position_m, dtype=float).reshape(3)
            contacted = False
            # Do **not** move the visual marker before planning. The sphere only
            # relocates when: (1) PLAN_OK — red, then EE moves; or (2) planning
            # fails after recovery timeout/exhaustion — yellow. That avoids the
            # confusing "target teleports, arm frozen" sequence.

            if reset_home:
                # Independent episode: return to home before planning so the
                # start pose is not the previous IK goal (fewer path-dependent
                # PLAN_FAIL / marker immersions from awkward starts).
                _move_joints_at_hardware_speed(
                    articulation,
                    q_home,
                    simulation_app,
                    max_speed_rad_s=max_speed,
                )
                q_now = q_home.copy()
            else:
                try:
                    q_now = _get_joint_positions(articulation)
                except Exception:
                    q_now = q_home.copy()

            # Visual marker radius for keepout / contact; planning uses a mild
            # inflation so cuRobo keeps the proximal arm clear of the *visual*
            # 12 mm sphere (mesh under-approx otherwise looks like arm-side hits).
            plan_inflate_m = float(
                plan_cfg.get("target_obstacle_inflate_m", 0.008)
            )
            visual_obstacle = SphereObstacle(
                center_m=target_xyz,
                radius_m=float(TARGET_MARKER_RADIUS_M),
                name="ik_target_visual",
            )
            target_obstacle = SphereObstacle(
                center_m=target_xyz,
                radius_m=float(TARGET_MARKER_RADIUS_M) + plan_inflate_m,
                name="ik_target",
            )

            # Prefilter: skip targets inside the base-column keepout. cuRobo's
            # mesh-fitted spheres + marker OBB collide at home for these poses
            # (NumPy capsules under-approximate the base), always yielding
            # INVALID_START_STATE_WORLD_COLLISION. Also skip capsule overlap.
            radial_xy = float(np.hypot(target_xyz[0], target_xyz[1]))
            in_keepout = (
                radial_xy < float(TARGET_MARKER_BASE_KEEPOUT_XY_M)
                and float(target_xyz[2]) < float(TARGET_MARKER_BASE_KEEPOUT_Z_M)
            )
            home_capsules = link_capsules_from_q(q_now)
            capsule_hit = any(
                capsule_sphere_collide(c, visual_obstacle) for c in home_capsules
            )
            if in_keepout or capsule_hit:
                n_skip += 1
                _viz_log(
                    f"  SKIP_OVERLAPPING_TARGET: marker at "
                    f"({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f}) "
                    f"radial_xy={radial_xy:.3f} keepout={in_keepout} "
                    f"capsule={capsule_hit} — not counted as an episode",
                    level="warn",
                )
                _viz_log(
                    f"{cand} RESULT SKIPPED overlapping_target | {_tally()}",
                    level="warn",
                )
                continue

            # Dexterity prescreen (recommended step #1): deterministically test
            # whether the oriented pad-facing contact pose is IK-reachable at all
            # (classical DLS IK, both tool-axis signs, cone orientations, several
            # seeds). Provably-infeasible targets are sampler optimism, not
            # planner faults — report SKIPPED_UNREACHABLE and exclude from the
            # rate gate / episode count. STRICT so it does not mask genuine
            # PLAN_FAILs.
            if prescreen_enabled:
                dex = contact_pose_is_dexterous(
                    target_xyz,
                    float(TARGET_MARKER_RADIUS_M),
                    cone_max_rad=cone_max_rad,
                    n_tilts=cone_tilts,
                    n_azimuths=cone_azimuths,
                    position_tol_m=prescreen_pos_tol,
                    orientation_tol_rad=prescreen_ori_tol,
                    n_seeds=prescreen_seeds,
                    region_margin_m=region_margin_m,
                    skip_orientation_infeasible=prescreen_skip_orientation,
                    backend=prescreen_backend,
                )
                if not dex.feasible:
                    n_skip += 1
                    n_skip_unreachable += 1
                    result_line = (
                        f"{cand} RESULT SKIPPED_UNREACHABLE "
                        f"{dex.as_log_tokens()} "
                        f"target=({target_xyz[0]:.3f},{target_xyz[1]:.3f},"
                        f"{target_xyz[2]:.3f}) | {_tally()}"
                    )
                    skipped_unreachable_log.append(result_line)
                    _viz_log(
                        f"  DEXTERITY_PRESCREEN: contact pose infeasible "
                        f"({dex.classification}; {dex.note}; region={dex.region_reason})"
                        " — excluded from episode count and rate gate",
                        level="warn",
                    )
                    _viz_log(result_line, level="warn")
                    continue

            # Countable episode: passed skip filters → consumes a visualize slot.
            n_countable += 1
            episode = f"[{n_countable}/{n_countable_target}]"
            _viz_log(
                f"{episode} EPISODE trial={trial.index} "
                f"target=({target_xyz[0]:.3f},{target_xyz[1]:.3f},"
                f"{target_xyz[2]:.3f})"
            )

            marker_committed = False
            contacted = False
            # Approach ray for tip-face contact (meters): tip at trial start, then
            # refreshed at the start of each executed segment so side grazes after
            # a lateral recovery approach do not turn the marker green.
            approach_from_m = np.asarray(
                forward_kinematics(q_now).position_m, dtype=float
            ).reshape(3).copy()
            # Per-episode contact diagnostics. ``logged_reasons`` de-dupes
            # servo-tick rejection spam; ``nearest`` keeps the closest sample so
            # a PLAN_FAIL(no_contact) can report *why* (through / side / axis).
            contact_diag = {
                "logged_reasons": set(),
                "nearest_dist": float("inf"),
                "nearest": None,
                "arm_swept": False,
                "mid_path_side_or_back": False,
                "mid_path_reject_metrics": None,
                "green_after_graze": False,
                "q_at_contact": None,
            }
            # Detect the "EE already close to target" regime up front. When the
            # tip starts within a short shell of the surface, the oriented
            # approach frequently needs many standoff vias (it must first back
            # off to a pad-facing standoff, then nudge in). Flag it so a high
            # retry count is attributable rather than mysterious.
            tip_start_dist = float(np.linalg.norm(approach_from_m - target_xyz))
            ee_close_shell_m = float(TARGET_MARKER_RADIUS_M) + 0.05
            ee_close_start = tip_start_dist <= ee_close_shell_m
            if ee_close_start:
                _viz_log(
                    f"  EE_CLOSE: tip starts {tip_start_dist * 1e3:.0f}mm from "
                    f"center (≤{ee_close_shell_m * 1e3:.0f}mm) — high via/retry "
                    "count expected (must back off to pad-facing standoff first)",
                    level="warn",
                )

            def _on_step(q_rad: np.ndarray, *, _tgt=target_xyz) -> None:
                nonlocal contacted
                _update_collision_spheres(q_rad)
                if contacted:
                    # #region agent log
                    if not contact_diag.get("logged_skip_after_green"):
                        contact_diag["logged_skip_after_green"] = True
                        _agent_debug_log(
                            "E",
                            "run_ik_viz.py:_on_step:early_return",
                            "further_midpath_checks_disabled_after_green",
                            {
                                "episode": episode,
                                "prior_graze": bool(
                                    contact_diag.get("mid_path_side_or_back")
                                ),
                            },
                        )
                    # #endregion
                    return
                # Mid-path: proximal arm must not graze the marker before the
                # tip-face lands (operator: "target colliding with the side of
                # the arm"). Distal EE capsules are ignored — see
                # proximal_arm_contacts_target.
                try:
                    arm_mid = proximal_arm_contacts_target(
                        q_rad,
                        _tgt,
                        target_radius_m=float(TARGET_MARKER_RADIUS_M),
                        link_radius_m=float(
                            plan_cfg.get(
                                "arm_sweep_link_radius_m",
                                plan_cfg.get("link_radius_m", 0.012),
                            )
                        ),
                        n_ee_segments_ignored=int(
                            plan_cfg.get("arm_sweep_n_ee_segments_ignored", 3)
                        ),
                    )
                    if arm_mid.collides and not contact_diag["arm_swept"]:
                        contact_diag["arm_swept"] = True
                        _viz_log(
                            "  MARKER_ARM_SWEEP: proximal arm link(s) intersect "
                            "the target before tip-face contact "
                            f"(reasons={list(arm_mid.reasons)}) — will fail "
                            "settle even if tip later greens",
                            level="error",
                        )
                except Exception:
                    pass
                pose = forward_kinematics(q_rad)
                ee = pose.position_m
                # Honest tip-face / surface-shell gate: reject side, through,
                # immersed (colliding into the volume), and wrong-side axis.
                ok, reason, metrics = classify_tip_contact(
                    ee,
                    _tgt,
                    approach_from_m=approach_from_m,
                    ee_quaternion_wxyz=pose.quaternion_wxyz,
                )
                if metrics["dist_m"] < contact_diag["nearest_dist"]:
                    contact_diag["nearest_dist"] = metrics["dist_m"]
                    contact_diag["nearest"] = (reason, dict(metrics))
                if ok:
                    if contact_diag.get("mid_path_side_or_back"):
                        contact_diag["green_after_graze"] = True
                    # #region agent log
                    _agent_debug_log(
                        "B",
                        "run_ik_viz.py:_on_step:green",
                        "midpath_green_accepted",
                        {
                            "episode": episode,
                            "prior_graze": bool(
                                contact_diag.get("mid_path_side_or_back")
                            ),
                            "prior_reasons": sorted(
                                contact_diag.get("logged_reasons", set())
                            ),
                            "dist_mm": float(metrics["dist_m"]) * 1e3,
                            "lat_mm": float(
                                metrics.get("lateral_m", float("nan"))
                            )
                            * 1e3,
                            "axis_out_deg": float(
                                np.degrees(
                                    metrics.get(
                                        "axis_out_err_rad", float("nan")
                                    )
                                )
                            ),
                        },
                    )
                    # #endregion
                    contacted = True
                    # Latch joints at first honest tip-face green. Later freeze /
                    # settle often sees tip back at standoff (20 mm) after tip-omit
                    # or PD drift (iter15 Ep8: green 13.5 mm → settle 20.4 mm).
                    try:
                        contact_diag["q_at_contact"] = np.asarray(
                            q_rad, dtype=float
                        ).reshape(6).copy()
                    except Exception:
                        contact_diag["q_at_contact"] = None
                    _set_target_marker_color(stage, state=MarkerVisualState.CONTACT)
                    _viz_log(
                        "  MARKER_CONTACT: tip-face center on sphere surface → green "
                        f"(dist={metrics['dist_m'] * 1e3:.1f}mm "
                        f"pen={metrics['penetration_m'] * 1e3:.1f}mm "
                        f"lat={metrics['lateral_m'] * 1e3:.1f}mm "
                        f"axis_in={np.degrees(metrics['axis_in_err_rad']):.0f}deg "
                        f"axis_out={np.degrees(metrics['axis_out_err_rad']):.0f}deg)"
                    )
                    return
                # Near-miss instrumentation: log each distinct failure mode once
                # per episode when the tip is actually near the surface.
                if (
                    reason != "no_contact"
                    and reason not in contact_diag["logged_reasons"]
                ):
                    contact_diag["logged_reasons"].add(reason)
                    if reason == "side_graze":
                        contact_diag["mid_path_side_or_back"] = True
                        contact_diag["mid_path_reject_metrics"] = {
                            "reason": reason,
                            "dist_mm": float(metrics["dist_m"]) * 1e3,
                            "lat_mm": float(
                                metrics.get("lateral_m", float("nan"))
                            )
                            * 1e3,
                            "axis_out_deg": float(
                                np.degrees(
                                    metrics.get(
                                        "axis_out_err_rad", float("nan")
                                    )
                                )
                            ),
                        }
                    elif reason == "wrong_side_axis":
                        # Latch only clear side/barrel (~90°) or flipped/back
                        # (~180°). Near-tol tip-face misses (e.g. axis_out≈15–20°
                        # just over TOOL_AXIS_TOL) are settled by CONTACT_HOLD —
                        # mid-path latching those caused false PLAN_FAIL after
                        # honest greens (iter9 Ep8).
                        axis_out = float(
                            metrics.get("axis_out_err_rad", float("nan"))
                        )
                        latch_axis = max(
                            0.50,
                            2.0 * float(TARGET_MARKER_TOOL_AXIS_TOL_RAD),
                        )
                        if np.isfinite(axis_out) and axis_out > latch_axis:
                            contact_diag["mid_path_side_or_back"] = True
                            contact_diag["mid_path_reject_metrics"] = {
                                "reason": reason,
                                "dist_mm": float(metrics["dist_m"]) * 1e3,
                                "lat_mm": float(
                                    metrics.get("lateral_m", float("nan"))
                                )
                                * 1e3,
                                "axis_out_deg": float(np.degrees(axis_out)),
                            }
                    if contact_diag.get("mid_path_side_or_back") and reason in (
                        "side_graze",
                        "wrong_side_axis",
                    ):
                        # #region agent log
                        _agent_debug_log(
                            "A",
                            "run_ik_viz.py:_on_step:graze",
                            "midpath_graze_latched_for_settle_fail",
                            {
                                "episode": episode,
                                "reason": reason,
                                "sets_final_fail_at_settle": True,
                                "dist_mm": float(metrics["dist_m"]) * 1e3,
                                "lat_mm": float(
                                    metrics.get("lateral_m", float("nan"))
                                )
                                * 1e3,
                                "axis_out_deg": float(
                                    np.degrees(
                                        metrics.get(
                                            "axis_out_err_rad", float("nan")
                                        )
                                    )
                                ),
                            },
                        )
                        # #endregion
                    label = {
                        "through": (
                            "MARKER_THROUGH: tip crossed to far hemisphere "
                            "(passed through marker) — not green"
                        ),
                        "immersed": (
                            "MARKER_IMMERSED: tip collided into the sphere "
                            "volume (not surface touch) — not green"
                        ),
                        "side_graze": (
                            "MARKER_SIDE_GRAZE: contact off the tip-face pad "
                            "(lateral) — will fail settle even if tip later greens"
                        ),
                        "wrong_side_axis": (
                            "MARKER_WRONG_SIDE: flange +Z not aligned with the "
                            "outward approach ray — contact on the side/barrel "
                            "or flipped/back onto the marker — will fail settle "
                            "even if tip later greens"
                        ),
                    }.get(reason, f"MARKER_REJECT:{reason}")
                    _viz_log(
                        f"  {label} "
                        f"(dist={metrics['dist_m'] * 1e3:.1f}mm "
                        f"pen={metrics['penetration_m'] * 1e3:.1f}mm "
                        f"lat={metrics['lateral_m'] * 1e3:.1f}mm "
                        f"axis_in={np.degrees(metrics['axis_in_err_rad']):.0f}deg "
                        f"axis_out={np.degrees(metrics['axis_out_err_rad']):.0f}deg)",
                        level="warn",
                    )

            def _execute_waypoints(waypoints_rad: np.ndarray, dt_s: float) -> None:
                """Move the EE during recovery (via1 / direct); commit red marker."""
                nonlocal marker_committed, approach_from_m
                wp = np.asarray(waypoints_rad, dtype=float)
                if wp.ndim == 1:
                    wp = wp.reshape(1, -1)
                if wp.shape[0] >= 1:
                    approach_from_m = np.asarray(
                        forward_kinematics(wp[0]).position_m, dtype=float
                    ).reshape(3).copy()
                if not marker_committed:
                    # First committed motion toward this goal — show red target.
                    _set_target_marker(
                        stage, target_xyz, state=MarkerVisualState.PENDING
                    )
                    marker_committed = True
                    _viz_log("  MARKER_COMMIT: red target (EE recovery motion starts)")
                _follow_trajectory(
                    articulation,
                    wp,
                    simulation_app,
                    dt_s=float(dt_s),
                    max_speed_rad_s=max_speed,
                    on_step=_on_step,
                )

            def _decision_emit(line: str) -> None:
                """Live DEC|… lines for smoke monitoring (grep-friendly)."""
                _viz_log(f"  {line}", level="warn")

            # Optional shorter recovery budget for fast iteration
            # (ISAAC_VIZ_RECOVERY_TIMEOUT_S); default = YAML 90 s.
            recovery_timeout = None
            env_to = os.environ.get("ISAAC_VIZ_RECOVERY_TIMEOUT_S", "").strip()
            if env_to:
                try:
                    recovery_timeout = float(env_to)
                except ValueError:
                    recovery_timeout = None

            traj = plan_collision_free_with_recovery(
                q_now,
                trial.q_sol,
                prefer_curobo=prefer_curobo,
                # Volumetric target: fitted EE/arm spheres must not intersect it.
                # Tip plans to marker surface; tip contact (red→green) stays valid.
                # Recovery loops until plan_recovery_timeout_s; executes via1
                # partials so the EE moves instead of freezing until full success.
                obstacles=[target_obstacle],
                planner=curobo_planner,
                contact_planner=curobo_contact_planner,
                execute_waypoints=_execute_waypoints,
                decision_emit=_decision_emit,
                timeout_s=recovery_timeout,
            )
            # Defense in depth: refuse historical ok_fallback|plan_failed successes.
            executable = may_execute_motion(
                plan_result_is_executable(traj), gate_on_failure=gate
            )
            used_via = (
                "strategy=via_standoff" in str(traj.message)
                or "strategy=via_contact" in str(traj.message)
            )
            if executable:
                n_plan_ok += 1
                if not marker_committed:
                    # Full plan returned without mid-recovery exec (rare).
                    _set_target_marker(
                        stage, target_xyz, state=MarkerVisualState.PENDING
                    )
                    marker_committed = True
                _viz_log(
                    f"  PLAN_OK backend={traj.backend} T={traj.waypoints_rad.shape[0]} "
                    f"dt_s={traj.dt_s:.4f} msg={traj.message}"
                )
                if used_via:
                    # Surface in the Isaac Sim GUI (Window → Console shows
                    # carb.log_warn in yellow): the direct plan failed and the
                    # target was only reachable through intermediate standoff
                    # waypoint(s) — worth operator attention even on success.
                    n_via_ok += 1
                    # Success messages carry a `via_attempts=N` field (failed
                    # ones record `via1_` markers — recovery_via_attempts_...).
                    via_m = re.search(r"via_attempts=(\d+)", str(traj.message))
                    via_n = via_m.group(1) if via_m else "?"
                    _viz_log(
                        f"  VIA_WAYPOINT_USED: target unreachable by direct "
                        f"plan — reached via intermediate standoff waypoint(s) "
                        f"(via_attempts={via_n})",
                        level="warn",
                    )
                    # Attribute a high retry count to the EE-close regime so it
                    # is diagnosable (see EE_CLOSE above), not mysterious.
                    try:
                        via_i = int(via_n)
                    except (TypeError, ValueError):
                        via_i = 0
                    if ee_close_start and via_i >= HIGH_RETRY_VIA_THRESHOLD:
                        _viz_log(
                            "  HIGH_RETRY_WHEN_CLOSE: EE started near the target "
                            f"({tip_start_dist * 1e3:.0f}mm) and needed "
                            f"{via_i} standoff vias (≥{HIGH_RETRY_VIA_THRESHOLD}) "
                            "— back-off-then-approach dominates the recovery",
                            level="warn",
                        )
            else:
                n_plan_fail += 1
                # Yellow only after the recovery timeout budget is exhausted.
                _set_target_marker(stage, target_xyz, state=MarkerVisualState.PLAN_FAIL)
                vias = recovery_via_attempts_in_message(traj.message)
                msg_short = str(traj.message)
                if len(msg_short) > 220:
                    msg_short = msg_short[:180] + "…(see DEC| lines above)"
                _viz_log(
                    f"  PLAN_FAIL backend={traj.backend} via_attempts={vias} "
                    f"msg={msg_short} (gate={gate}; marker=yellow after timeout)",
                    level="warn",
                )
                if ee_close_start and vias >= HIGH_RETRY_VIA_THRESHOLD:
                    _viz_log(
                        "  HIGH_RETRY_WHEN_CLOSE: EE started near the target "
                        f"({tip_start_dist * 1e3:.0f}mm) and burned {vias} "
                        f"standoff vias (≥{HIGH_RETRY_VIA_THRESHOLD}) before "
                        "timeout — back-off-then-approach dominates the recovery",
                        level="warn",
                    )
                if vias < 1:
                    _viz_log(
                        "  RECOVERY_AUDIT: no via1_ attempt in fail message "
                        "(unexpected when plan_recovery_enabled)",
                        level="error",
                    )
                else:
                    _viz_log(
                        f"  RECOVERY: tried {vias} via standoff(s); timeout exhausted "
                        "(yellow only after plan_recovery_timeout_s)",
                        level="warn",
                    )
                # Freeze current pose: clear any prior PD targets so the arm
                # cannot keep servo-ing while the yellow marker is displayed.
                try:
                    q_freeze = _get_joint_positions(articulation)
                    _set_joint_positions(articulation, q_freeze)
                except Exception:
                    pass
                _viz_log("  GATED_NO_MOTION: holding pose (plan rejected)", level="warn")
                _viz_log(
                    f"{episode} RESULT PLAN_FAIL via_attempts={vias} | {_tally()}",
                    level="warn",
                )
                t_gate = time.monotonic() + max(0.05, float(hold_s))
                while time.monotonic() < t_gate and simulation_app.is_running():
                    simulation_app.update()
                if not simulation_app.is_running():
                    _viz_log(
                        "  GUI_STOP: simulation_app stopped during PLAN_FAIL hold; "
                        "ending trial loop early",
                        level="warn",
                    )
                    break
                # Early abort: no greens after N consecutive fails — contact
                # stack is systematically broken; do not burn ~90s/episode.
                early_n = int(getattr(args, "early_abort_after_fails", 3) or 0)
                if early_n > 0 and n_plan_fail >= early_n:
                    _viz_log(
                        f"  EARLY_ABORT: {n_plan_fail} PLAN_FAIL "
                        f"(threshold={early_n}, ok={n_plan_ok}) — stopping viz "
                        "loop for faster iteration (spec: abort on failure).",
                        level="error",
                    )
                    break
                continue

            # After recovery / contact legs, re-anchor the tip-face gate on the
            # radial through the current tip (oriented contact axis). Using a
            # stale via-start approach ray falsely rejects tip-face hits after
            # multi-via recovery.
            try:
                from residual_adaptive_ik.planning.contact_geometry import (
                    build_sphere_contact_approach,
                )

                q_gate = _get_joint_positions(articulation)
                tip_gate = np.asarray(
                    forward_kinematics(q_gate).position_m, dtype=float
                ).reshape(3)
                ca_gate = build_sphere_contact_approach(
                    tip_gate,
                    target_xyz,
                    float(TARGET_MARKER_RADIUS_M),
                    standoff_m=0.02,
                    nudge_m=0.008,
                    current_quaternion_wxyz=forward_kinematics(
                        q_gate
                    ).quaternion_wxyz,
                )
                approach_from_m = np.asarray(
                    ca_gate.standoff_position_m, dtype=float
                ).reshape(3).copy()
            except Exception:
                pass

            # Remaining waypoints (hold row if already_executed during recovery).
            if (
                traj.waypoints_rad is not None
                and int(getattr(traj.waypoints_rad, "size", 0)) > 0
                and "already_executed" not in str(traj.message)
            ):
                _follow_trajectory(
                    articulation,
                    traj.waypoints_rad,
                    simulation_app,
                    dt_s=traj.dt_s,
                    max_speed_rad_s=max_speed,
                    on_step=_on_step,
                )
            elif "already_executed" in str(traj.message):
                # Nudge one servo step so contact/hold still samples FK.
                try:
                    q_hold = _get_joint_positions(articulation)
                    _set_joint_positions(articulation, q_hold)
                    _on_step(q_hold)
                except Exception:
                    pass
            # Freeze joints through the settle hold. Prefer the first green
            # contact pose when the tip has already drifted back toward standoff
            # (iter15 Ep8: green 13.5 mm → freeze would otherwise capture 20.4 mm).
            q_freeze = None
            restored_green = False
            try:
                q_now_hold = np.asarray(
                    _get_joint_positions(articulation), dtype=float
                ).reshape(6).copy()
                tip_now_hold = np.asarray(
                    forward_kinematics(q_now_hold).position_m, dtype=float
                ).reshape(3)
                d_hold = float(np.linalg.norm(tip_now_hold - target_xyz))
                shell_max = (
                    float(TARGET_MARKER_CONTACT_DISTANCE_M)
                    + float(TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M)
                )
                q_green = contact_diag.get("q_at_contact")
                if (
                    q_green is not None
                    and d_hold > shell_max + 5e-4  # 0.5 mm past shell
                    and np.asarray(q_green).shape == (6,)
                ):
                    q_freeze = np.asarray(q_green, dtype=float).reshape(6).copy()
                    restored_green = True
                    _set_joint_positions(articulation, q_freeze)
                    _on_step(q_freeze)
                    _viz_log(
                        "  SETTLE_RESTORE: tip at "
                        f"{d_hold * 1e3:.1f}mm (>shell {shell_max * 1e3:.1f}mm); "
                        "restoring q_at_contact from first green"
                    )
                else:
                    q_freeze = q_now_hold
            except Exception:
                q_freeze = None
                restored_green = False
            t_end = time.monotonic() + max(0.05, float(hold_s))
            while time.monotonic() < t_end and simulation_app.is_running():
                try:
                    if q_freeze is not None:
                        _set_joint_positions(articulation, q_freeze)
                        _on_step(q_freeze)
                    else:
                        q_hold = _get_joint_positions(articulation)
                        _on_step(q_hold)
                except Exception:
                    pass
                simulation_app.update()

            # Axial pierce refine when tip is outside the surface shell — even if
            # a mid-path green already latched ``contacted``. Iter6: green at
            # 13.6 mm then settle 14.0 mm (just past outer_tol) with nan axes.
            # Also refine when tip is in-shell but pad axis is past the tip-face
            # tol (iter7: dist OK after refine but axis_out≈37° with quat_now).
            # Skip refine after SETTLE_RESTORE — green pose already validated
            # mid-path; CONTACT_HOLD from standoff caused iter15 Ep8 no_contact.
            def _needs_contact_hold_refine() -> bool:
                try:
                    q_chk = _get_joint_positions(articulation)
                    p_chk = forward_kinematics(q_chk)
                    tip_chk = np.asarray(p_chk.position_m, dtype=float).reshape(3)
                    fok, freason, _fm = classify_tip_contact(
                        tip_chk,
                        target_xyz,
                        approach_from_m=approach_from_m,
                        ee_quaternion_wxyz=p_chk.quaternion_wxyz,
                    )
                    if fok:
                        return False
                    return freason in (
                        "no_contact",
                        "wrong_side_axis",
                        "side_graze",
                    )
                except Exception:
                    return not contacted

            if (not restored_green) and (
                (not contacted) or _needs_contact_hold_refine()
            ):
                # Short axial refine toward the pierce point (surface), not the
                # marker center. Keeps tip-face contact without reintroducing
                # side/flange immersion from a center-drive to trial.q_sol.
                from isaac_sim.target_marker import tip_face_pierce_point_m

                q_now = _get_joint_positions(articulation)
                tip_now = np.asarray(
                    forward_kinematics(q_now).position_m, dtype=float
                ).reshape(3)
                pierce = tip_face_pierce_point_m(tip_now, target_xyz)
                tip_dist = float(np.linalg.norm(tip_now - target_xyz))
                _viz_log(
                    "  CONTACT_HOLD: axial refine to pierce "
                    f"(tip_to_center={tip_dist:.4f} m, "
                    f"tip_to_pierce={float(np.linalg.norm(tip_now - pierce)):.4f} m)"
                )
                # Gate contact along the refine axis (standoff → center).
                approach_from_m = tip_now.copy()
                try:
                    from residual_adaptive_ik.kinematics.fk import Pose
                    from residual_adaptive_ik.kinematics.numerical_ik import (
                        DampedLeastSquaresIK,
                    )
                    from residual_adaptive_ik.planning.contact_geometry import (
                        build_sphere_contact_approach,
                    )

                    # Pad-facing quat at pierce (not current wrist). Keeping
                    # quat_now after a sequential handoff left settle at
                    # axis_out≈37° (iter7) even when tip was on the shell.
                    ca_hold = build_sphere_contact_approach(
                        tip_now,
                        target_xyz,
                        float(TARGET_MARKER_RADIUS_M),
                        standoff_m=0.008,
                        nudge_m=0.008,
                        current_quaternion_wxyz=forward_kinematics(
                            q_now
                        ).quaternion_wxyz,
                    )
                    quat_goal = np.asarray(
                        ca_hold.quaternion_wxyz, dtype=float
                    ).reshape(4)
                    ik = DampedLeastSquaresIK(
                        max_iterations=120,
                        damping=1e-3,
                        position_tol_m=5e-4,
                        orientation_tol_rad=0.04,
                    )
                    res = ik.solve(
                        Pose(position_m=pierce, quaternion_wxyz=quat_goal),
                        seed_q=q_now,
                    )
                    hold_ok = bool(getattr(res, "success", False))
                    if not hold_ok:
                        # Position-first fallback (iter15 Ep8: pad-facing IK
                        # failed at tip_to_pierce=8.4 mm from standoff).
                        ik_pos = DampedLeastSquaresIK(
                            max_iterations=160,
                            damping=5e-3,
                            position_tol_m=1.5e-3,
                            orientation_tol_rad=0.35,
                        )
                        res = ik_pos.solve(
                            Pose(
                                position_m=pierce,
                                quaternion_wxyz=quat_goal,
                            ),
                            seed_q=q_now,
                        )
                        hold_ok = bool(getattr(res, "success", False))
                        if hold_ok:
                            _viz_log(
                                "  CONTACT_HOLD: position-relaxed IK ok "
                                "(orientation_tol=0.35 rad)"
                            )
                    if hold_ok:
                        tip_chk = np.asarray(
                            forward_kinematics(res.q).position_m, dtype=float
                        ).reshape(3)
                        tip_err = float(np.linalg.norm(tip_chk - pierce))
                        if tip_err > 0.005:
                            _viz_log(
                                "  CONTACT_HOLD: IK tip_err_m="
                                f"{tip_err:.4f} > 5mm — rejecting solution",
                                level="warn",
                            )
                            hold_ok = False
                    if hold_ok:
                        _move_joints_at_hardware_speed(
                            articulation,
                            res.q,
                            simulation_app,
                            max_speed_rad_s=max_speed,
                            on_step=_on_step,
                        )
                        q_freeze = np.asarray(res.q, dtype=float).reshape(6).copy()
                        approach_from_m = np.asarray(
                            ca_hold.standoff_position_m, dtype=float
                        ).reshape(3).copy()
                    else:
                        q_green = contact_diag.get("q_at_contact")
                        if q_green is not None:
                            _viz_log(
                                "  CONTACT_HOLD: IK fail — restoring q_at_contact",
                                level="warn",
                            )
                            _move_joints_at_hardware_speed(
                                articulation,
                                q_green,
                                simulation_app,
                                max_speed_rad_s=max_speed,
                                on_step=_on_step,
                            )
                            q_freeze = np.asarray(
                                q_green, dtype=float
                            ).reshape(6).copy()
                        else:
                            _viz_log(
                                "  CONTACT_HOLD: axial IK did not converge "
                                f"({getattr(res, 'reason', 'unknown')})"
                            )
                except Exception as exc:  # noqa: BLE001
                    _viz_log(f"  CONTACT_HOLD: axial IK skipped ({exc})")
                t_nudge = time.monotonic() + max(0.15, float(hold_s))
                while time.monotonic() < t_nudge and simulation_app.is_running():
                    try:
                        if q_freeze is not None:
                            _set_joint_positions(articulation, q_freeze)
                            _on_step(q_freeze)
                        else:
                            _on_step(_get_joint_positions(articulation))
                    except Exception:
                        pass
                    simulation_app.update()
            # Authoritative final-pose contact check. The marker may have
            # flashed green on a transient sample during motion; the **settled**
            # pose must be a valid *front* tip-face contact. A wrong-side /
            # inside-EE / through settled contact is reported as a FAILURE even
            # though it turned green (independent monitor, per operator request).
            final_ok = bool(contacted)
            fail_reason_tag = "no_contact"
            q_settled = None
            try:
                q_settled = _get_joint_positions(articulation)
            except Exception:
                q_settled = None
            if contacted and q_settled is not None:
                try:
                    p_fin = forward_kinematics(q_settled)
                    tip_fin = np.asarray(p_fin.position_m, dtype=float).reshape(3)
                    fok, freason, fm = classify_tip_contact(
                        tip_fin,
                        target_xyz,
                        approach_from_m=approach_from_m,
                        ee_quaternion_wxyz=p_fin.quaternion_wxyz,
                    )
                    if not fok:
                        final_ok = False
                        fail_reason_tag = (
                            "immersed"
                            if freason == "immersed"
                            else (
                                "no_contact"
                                if freason == "no_contact"
                                else "invalid_side"
                            )
                        )
                        n_invalid_side += 1
                        _set_target_marker_color(
                            stage, state=MarkerVisualState.PLAN_FAIL
                        )
                        _viz_log(
                            "  CONTACT_INVALID_SETTLE: marker turned green during "
                            "motion, but the settled contact is invalid "
                            f"(reason={freason}, "
                            f"dist={fm['dist_m'] * 1e3:.1f}mm "
                            f"axis_in={np.degrees(fm['axis_in_err_rad']):.0f}deg "
                            f"axis_out={np.degrees(fm['axis_out_err_rad']):.0f}deg "
                            f"pen={fm['penetration_m'] * 1e3:.1f}mm "
                            f"lat={fm['lateral_m'] * 1e3:.1f}mm) — reported as "
                            "PLAN_FAIL despite the green flash",
                            level="error",
                        )
                    # Volumetric EE-body check: tip-zone sphere ∩ marker is OK
                    # at contact; any *side* sphere hit means the barrel/flange
                    # clipped the marker (false-green under tip-omit). Pure
                    # YAML spheres + NumPy FK — same envelope as planning diag.
                    try:
                        from residual_adaptive_ik.planning.collision_sphere_world import (
                            collision_spheres_in_base_frame,
                        )
                        from residual_adaptive_ik.planning.marker_contact_diag import (
                            settle_has_side_sphere_hits,
                        )

                        worlds = collision_spheres_in_base_frame(q_settled)
                        xyzr = np.asarray(
                            [
                                [
                                    float(s.center_m[0]),
                                    float(s.center_m[1]),
                                    float(s.center_m[2]),
                                    float(s.radius_m),
                                ]
                                for s in worlds
                            ],
                            dtype=float,
                        ).reshape(-1, 4)
                        approach_settle = (
                            None
                            if approach_from_m is None
                            else (
                                target_xyz
                                - np.asarray(approach_from_m, dtype=float).reshape(3)
                            )
                        )
                        side_hit, side_rep = settle_has_side_sphere_hits(
                            xyzr,
                            tip_fin,
                            target_xyz,
                            marker_radius_m=float(TARGET_MARKER_RADIUS_M),
                            approach_m=approach_settle,
                        )
                        if side_hit:
                            _viz_log(
                                "  CONTACT_INVALID_SETTLE: "
                                "MARKER_EE_SIDE_SPHERE — settled pose has "
                                f"{side_rep.n_side_hits} side collision-sphere "
                                f"hit(s) on the marker (tip_hits="
                                f"{side_rep.n_tip_hits}); tip-face flash alone "
                                "does not count — PLAN_FAIL(invalid_side)",
                                level="error",
                            )
                            # Override only when tip classify still looked OK
                            # (the false-green case). Tip-already-failed keeps
                            # its reason; do not double-count n_invalid_side.
                            if final_ok:
                                final_ok = False
                                fail_reason_tag = "invalid_side"
                                n_invalid_side += 1
                                _set_target_marker_color(
                                    stage, state=MarkerVisualState.PLAN_FAIL
                                )
                        # #region agent log
                        _agent_debug_log(
                            "C",
                            "run_ik_viz.py:settle",
                            "settle_outcome",
                            {
                                "episode": episode,
                                "mid_path_graze": bool(
                                    contact_diag.get("mid_path_side_or_back")
                                ),
                                "green_after_graze": bool(
                                    contact_diag.get("green_after_graze")
                                ),
                                "mid_path_reject": contact_diag.get(
                                    "mid_path_reject_metrics"
                                ),
                                "tip_classify_ok": bool(fok),
                                "tip_reason": freason,
                                "tip_axis_out_deg": float(
                                    np.degrees(
                                        fm.get(
                                            "axis_out_err_rad", float("nan")
                                        )
                                    )
                                ),
                                "tip_lat_mm": float(
                                    fm.get("lateral_m", float("nan"))
                                )
                                * 1e3,
                                "side_sphere_hit": bool(side_hit),
                                "n_side_hits": int(
                                    getattr(side_rep, "n_side_hits", -1)
                                ),
                                "n_tip_hits": int(
                                    getattr(side_rep, "n_tip_hits", -1)
                                ),
                                "final_ok_so_far": bool(final_ok),
                                "fail_reason_tag": fail_reason_tag,
                            },
                            run_id="post-fix",
                        )
                        # #endregion
                    except Exception as exc:  # noqa: BLE001
                        _viz_log(
                            f"  MARKER_EE_SIDE_SPHERE check skipped ({exc})",
                            level="warn",
                        )
                except Exception:
                    pass

            # EE-only contact: proximal arm / forearm must not intersect the
            # marker volume (settle) and must not have swept it mid-path.
            if contact_diag.get("arm_swept"):
                final_ok = False
                fail_reason_tag = "arm_body_contact"
                _set_target_marker_color(stage, state=MarkerVisualState.PLAN_FAIL)
                _viz_log(
                    "  CONTACT_INVALID_ARM_SWEEP: proximal arm intersected the "
                    "marker during the approach — PLAN_FAIL (EE tip-face "
                    "exclusive contact required)",
                    level="error",
                )
            # Mid-path tip-face side/back graze is latched like arm_swept: a later
            # pad-looking settle must not erase the visual side approach.
            if contact_diag.get("mid_path_side_or_back") and fail_reason_tag != (
                "arm_body_contact"
            ):
                graze_m = contact_diag.get("mid_path_reject_metrics") or {}
                if final_ok:
                    n_invalid_side += 1
                final_ok = False
                fail_reason_tag = "invalid_side"
                _set_target_marker_color(stage, state=MarkerVisualState.PLAN_FAIL)
                _viz_log(
                    "  CONTACT_INVALID_MIDPATH_GRAZE: mid-path "
                    f"{graze_m.get('reason', 'side/back')} "
                    f"(dist={graze_m.get('dist_mm', float('nan')):.1f}mm "
                    f"lat={graze_m.get('lat_mm', float('nan')):.1f}mm "
                    f"axis_out={graze_m.get('axis_out_deg', float('nan')):.0f}deg) "
                    "— PLAN_FAIL(invalid_side) even if tip later greened",
                    level="error",
                )
                # #region agent log
                _agent_debug_log(
                    "A",
                    "run_ik_viz.py:settle_midpath_latch",
                    "midpath_graze_forced_plan_fail",
                    {
                        "episode": episode,
                        "graze": graze_m,
                        "green_after_graze": bool(
                            contact_diag.get("green_after_graze")
                        ),
                        "fail_reason_tag": fail_reason_tag,
                    },
                    run_id="post-fix",
                )
                # #endregion
            if q_settled is not None and fail_reason_tag != "arm_body_contact":
                try:
                    arm_hit = proximal_arm_contacts_target(
                        q_settled,
                        target_xyz,
                        target_radius_m=float(TARGET_MARKER_RADIUS_M),
                        link_radius_m=float(
                            plan_cfg.get(
                                "arm_sweep_link_radius_m",
                                plan_cfg.get("link_radius_m", 0.012),
                            )
                        ),
                        n_ee_segments_ignored=int(
                            plan_cfg.get("arm_sweep_n_ee_segments_ignored", 3)
                        ),
                    )
                    if arm_hit.collides:
                        final_ok = False
                        fail_reason_tag = "arm_body_contact"
                        _set_target_marker_color(
                            stage, state=MarkerVisualState.PLAN_FAIL
                        )
                        _viz_log(
                            "  MARKER_ARM_BODY: proximal link(s) intersect the "
                            "target sphere (EE tip-face exclusive contact "
                            f"required); reasons={list(arm_hit.reasons)} — "
                            "PLAN_FAIL",
                            level="error",
                        )
                except Exception as exc:  # noqa: BLE001
                    _viz_log(
                        f"  MARKER_ARM_BODY check skipped ({exc})",
                        level="warn",
                    )

            if final_ok:
                n_contact_green += 1
                strat = "direct"
                if "via_contact" in str(traj.message):
                    strat = "via_contact"
                elif used_via:
                    strat = "via_standoff"
                _viz_log(
                    f"{episode} RESULT PLAN_OK "
                    f"strategy={strat} "
                    f"contact=green | {_tally()}",
                    level="warn" if used_via else "info",
                )
                # Sequential mode: clear the tip off this marker before the next
                # episode so the following approach does not mid-path graze with
                # a flipped wrist (CONTACT_INVALID_MIDPATH_GRAZE axis_out≈π).
                if not reset_home:
                    _sequential_retract_tip_from_marker(
                        articulation,
                        simulation_app,
                        target_xyz,
                        retract_m=0.05,
                        max_speed_rad_s=max_speed,
                    )
            else:
                # Reclassify: PLAN_OK without a valid settled contact is a failure.
                n_plan_ok -= 1
                n_plan_fail += 1
                _set_target_marker_color(stage, state=MarkerVisualState.PLAN_FAIL)
                if fail_reason_tag in (
                    "invalid_side",
                    "immersed",
                    "arm_body_contact",
                    "no_contact",
                ):
                    _viz_log(
                        f"{episode} RESULT PLAN_FAIL({fail_reason_tag}) | {_tally()}",
                        level="warn",
                    )
                else:
                    try:
                        tip_fail = np.asarray(
                            forward_kinematics(
                                _get_joint_positions(articulation)
                            ).position_m,
                            dtype=float,
                        ).reshape(3)
                        d_fail = float(np.linalg.norm(tip_fail - target_xyz))
                        # Report the *nearest* sample's failure mode (through /
                        # side_graze / wrong_side_axis / no_contact) so the
                        # operator sees why no valid tip-face contact occurred.
                        near = contact_diag.get("nearest")
                        near_reason = near[0] if near else "no_contact"
                        near_m = near[1] if near else {}
                        _viz_log(
                            "  MARKER_NO_CONTACT: PLAN_OK but tip never made a "
                            f"valid tip-face contact (nearest_reason={near_reason}, "
                            f"tip_to_center={d_fail:.4f} m, "
                            f"need≤{TARGET_MARKER_CONTACT_DISTANCE_M + TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M:.4f} m; "
                            f"nearest_dist={near_m.get('dist_m', float('nan')) * 1e3:.1f}mm "
                            f"pen={near_m.get('penetration_m', float('nan')) * 1e3:.1f}mm "
                            f"axis_in={np.degrees(near_m.get('axis_in_err_rad', float('nan'))):.0f}deg) "
                            "→ reclassified as PLAN_FAIL",
                            level="warn",
                        )
                    except Exception:
                        _viz_log(
                            "  MARKER_NO_CONTACT: PLAN_OK but tip never reached "
                            "sphere surface → reclassified as PLAN_FAIL",
                            level="warn",
                        )
                    _viz_log(
                        f"{episode} RESULT PLAN_FAIL(no_contact) | {_tally()}",
                        level="warn",
                    )

            # Repeated-via pressure across consecutive episodes (math metric).
            # Prefer the operator-facing recovery via count (matches the RESULT
            # line); fall back to the PLAN_OK ``via_attempts=N`` token.
            episode_via_attempts = recovery_via_attempts_in_message(traj.message)
            if episode_via_attempts < 1:
                m_via = re.search(r"via_attempts=(\d+)", str(traj.message))
                episode_via_attempts = int(m_via.group(1)) if m_via else 0
            vp = via_pressure.update(episode_via_attempts)
            _viz_log(
                f"  VIA_PRESSURE: this_via={episode_via_attempts} "
                f"ema_usage={vp['ema_usage']:.2f} "
                f"ema_attempts={vp['ema_attempts']:.1f} streak={vp['streak']}"
            )
            if vp["high"]:
                _viz_log(
                    "  VIA_PRESSURE_HIGH: vias needed across consecutive episodes "
                    f"(ema_usage={vp['ema_usage']:.2f}≥"
                    f"{via_pressure.ema_usage_threshold}, streak={vp['streak']}≥"
                    f"{via_pressure.streak_threshold}) — approaches are likely "
                    "systematically wrong-sided; check contact orientation",
                    level="warn",
                )

            if not simulation_app.is_running():
                _viz_log(
                    "  GUI_STOP: simulation_app stopped; ending trial loop early",
                    level="warn",
                )
                break

            # Fail-fast: also abort after settle-reclassified PLAN_FAIL
            # (mid-path graze / invalid settle), not only gated planning fails.
            early_n = int(getattr(args, "early_abort_after_fails", 3) or 0)
            if early_n > 0 and n_plan_fail >= early_n:
                _viz_log(
                    f"  EARLY_ABORT: {n_plan_fail} PLAN_FAIL "
                    f"(threshold={early_n}, ok={n_plan_ok}) — stopping viz loop.",
                    level="error",
                )
                break

        metrics["phase2_plan_ok"] = n_plan_ok
        metrics["phase2_plan_fail"] = n_plan_fail
        metrics["phase2_marker_contact_green"] = n_contact_green
        metrics["phase2_via_waypoint_ok"] = n_via_ok
        metrics["phase2_skipped_targets"] = n_skip
        metrics["phase2_skipped_unreachable"] = n_skip_unreachable
        metrics["phase2_countable_episodes"] = n_countable
        metrics["phase2_countable_target"] = n_countable_target
        metrics["phase2_candidates_considered"] = n_candidates_considered
        metrics["phase2_invalid_side_contacts"] = n_invalid_side
        metrics["phase2_via_pressure_ema_usage"] = float(
            via_pressure._ema_usage or 0.0
        )
        metrics["phase2_via_pressure_max_streak"] = int(via_pressure._max_streak)
        metrics["phase2_backend"] = (
            "curobo" if curobo_planner is not None else "numpy_lerp"
        )
        rate = plan_ok_rate(n_plan_ok, n_plan_fail)
        metrics["phase2_plan_ok_rate"] = rate
        skip_frac = skip_unreachable_frac(
            n_skip_unreachable, n_candidates_considered
        )
        metrics["phase2_skip_unreachable_frac"] = skip_frac
        # Resolve min rate: CLI > env > YAML (0 disables the gate).
        if args.min_plan_ok_rate is not None:
            min_rate = float(args.min_plan_ok_rate)
        elif os.environ.get("ISAAC_VIZ_MIN_PLAN_OK_RATE", "").strip() != "":
            min_rate = float(os.environ["ISAAC_VIZ_MIN_PLAN_OK_RATE"])
        else:
            min_rate = float(plan_cfg.get("min_plan_ok_rate", 0.25))
        metrics["phase2_min_plan_ok_rate"] = min_rate
        if args.max_skip_unreachable_frac is not None:
            max_skip_frac = float(args.max_skip_unreachable_frac)
        elif os.environ.get("ISAAC_VIZ_MAX_SKIP_UNREACHABLE_FRAC", "").strip() != "":
            max_skip_frac = float(os.environ["ISAAC_VIZ_MAX_SKIP_UNREACHABLE_FRAC"])
        else:
            max_skip_frac = float(plan_cfg.get("max_skip_unreachable_frac", 0.25))
        metrics["phase2_max_skip_unreachable_frac"] = max_skip_frac
        write_json(json_out, metrics)
        _viz_log(
            f"Phase 2 planning summary: ok={n_plan_ok} fail={n_plan_fail} "
            f"via={n_via_ok} skip={n_skip} skip_unreachable={n_skip_unreachable} "
            f"skip_frac={skip_frac:.3f} max_skip={max_skip_frac:.3f} "
            f"episodes={n_countable}/{n_countable_target} "
            f"candidates={n_candidates_considered} "
            f"rate={rate:.3f} min_required={min_rate:.3f} "
            f"marker_green={n_contact_green} "
            f"backend={metrics['phase2_backend']}",
            level="warn" if (n_plan_fail > 0 or n_via_ok > 0) else "info",
        )
        gate_ok = meets_min_plan_ok_rate(n_plan_ok, n_plan_fail, min_rate=min_rate)
        if not gate_ok:
            _viz_log(
                f"Phase 2 PLAN_OK rate gate FAILED: {rate:.3f} < {min_rate:.3f} "
                f"({n_plan_ok} ok / {n_plan_ok + n_plan_fail} planned)",
                level="error",
            )
        skip_gate_ok = meets_max_skip_unreachable_frac(
            n_skip_unreachable,
            n_candidates_considered,
            max_frac=max_skip_frac,
        )
        if not skip_gate_ok:
            _viz_log(
                f"Phase 2 SKIPPED_UNREACHABLE gate FAILED: "
                f"{skip_frac:.3f} > {max_skip_frac:.3f} "
                f"({n_skip_unreachable} unreachable / "
                f"{n_candidates_considered} candidates) — sampler is wasting "
                "budget outside the dexterous workspace",
                level="error",
            )
            gate_ok = False
        if n_countable < n_countable_target:
            _viz_log(
                f"Phase 2 episode-fill WARNING: only filled "
                f"{n_countable}/{n_countable_target} countable episodes "
                f"(candidate pool exhausted after {n_candidates_considered})",
                level="warn",
            )

        # End-of-test SKIPPED_UNREACHABLE analysis (spec.md Phase 2 requirement):
        # speculate why each target was skipped and, for Dexterous-Region cases,
        # a via that would support a deterministic IK. Print to the prompt output
        # AND append to STATUS.md.
        try:
            analyze_and_report(
                "\n".join(skipped_unreachable_log),
                status_path=REPO_ROOT / "STATUS.md",
                total_planned=n_plan_ok + n_plan_fail,
                print_fn=lambda s: _viz_log(s, level="warn"),
            )
        except Exception as exc:  # noqa: BLE001
            _viz_log(f"SKIPPED_UNREACHABLE analysis failed: {exc}", level="warn")

        print("Phase 1 metrics + visualization complete. Close Isaac Sim or Ctrl+C.")
        if not args.headless and not args.auto_exit:
            while simulation_app.is_running():
                simulation_app.update()
        elif not args.headless and args.auto_exit:
            print("NOTE: --auto-exit set; closing Kit after visualization (agent/GUI smoke).")
        return 0 if gate_ok else 2
    finally:
        simulation_app.close()


def main() -> int:
    args = parse_args()
    try:
        return run_viz(args)
    except ImportError as exc:
        print(
            "Isaac Sim Python modules are unavailable.\n"
            "Run with ${ISAACSIM_PATH}/python.sh on the Isaac Sim **host**.\n"
            "  ./scripts/host/run_isaac_viz.sh\n"
            f"ImportError: {exc}",
            file=sys.stderr,
        )
        return 1
    except Exception as exc:  # noqa: BLE001
        import traceback

        print(f"Phase 1 Isaac metrics+viz failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
