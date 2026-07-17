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

from isaac_sim.target_marker import (  # noqa: E402
    TARGET_MARKER_BASE_KEEPOUT_XY_M,
    TARGET_MARKER_BASE_KEEPOUT_Z_M,
    TARGET_MARKER_CONTACT_DISTANCE_M,
    TARGET_MARKER_RADIUS_M,
    TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
    classify_tip_contact,
    ee_contacts_target,
    marker_rgb_for_state,
)
from isaac_sim.viz_plan_policy import (  # noqa: E402
    MarkerVisualState,
    ViaPressureTracker,
    may_execute_motion,
    meets_min_plan_ok_rate,
    plan_ok_rate,
    plan_result_is_executable,
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
        to_show = select_trials_for_visualization(trials, max_visualize=int(args.visualize))
        metrics["isaac_viz"] = True
        metrics["n_visualized"] = len(to_show)
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
            f"Visualizing {len(to_show)}/{len(trials)} trials "
            f"(hold={hold_s:.3f}s wall, time_warp={time_warp:g}×; "
            f"sphere red→green on EE contact "
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

        def _tally() -> str:
            """One-line running status for real-time log monitoring.

            Grep-friendly: ``STATUS ok=.. fail=.. via=.. green=.. skip=..
            rate=..`` after every episode, so a tail of the log always shows
            the current totals without waiting for the end-of-run summary.
            """
            rate_now = plan_ok_rate(n_plan_ok, n_plan_fail)
            return (
                f"STATUS ok={n_plan_ok} fail={n_plan_fail} via={n_via_ok} "
                f"green={n_contact_green} skip={n_skip} rate={rate_now:.3f}"
            )
        q_home = load_home_joint_positions_rad()
        # CLI / smoke default is sequential: --no-reset-to-home (False).
        # Pass --reset-to-home only for the independent-episode 1.0 gate.
        reset_home = bool(args.reset_to_home)
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
        for i, trial in enumerate(to_show):
            status = "OK" if trial.success else f"FAIL({trial.reason})"
            episode = f"[{i + 1}/{len(to_show)}]"
            _viz_log(
                f"{episode} EPISODE trial={trial.index} ik={status} "
                f"pos_err_m={trial.position_error_m:.4e} "
                f"target=({trial.target.position_m[0]:.3f},"
                f"{trial.target.position_m[1]:.3f},"
                f"{trial.target.position_m[2]:.3f})"
            )
            if not trial.success:
                _viz_log(
                    f"{episode} RESULT IK_FAIL reason={trial.reason} | {_tally()}",
                    level="warn",
                )
                continue

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

            target_obstacle = SphereObstacle(
                center_m=target_xyz,
                radius_m=float(TARGET_MARKER_RADIUS_M),
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
                capsule_sphere_collide(c, target_obstacle) for c in home_capsules
            )
            if in_keepout or capsule_hit:
                n_skip += 1
                _viz_log(
                    f"  SKIP_OVERLAPPING_TARGET: marker at "
                    f"({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f}) "
                    f"radial_xy={radial_xy:.3f} keepout={in_keepout} "
                    f"capsule={capsule_hit} — not counted",
                    level="warn",
                )
                _viz_log(
                    f"{episode} RESULT SKIPPED overlapping_target | {_tally()}",
                    level="warn",
                )
                continue

            # Dexterity prescreen (recommended step #1): deterministically test
            # whether the oriented pad-facing contact pose is IK-reachable at all
            # (classical DLS IK, both tool-axis signs, cone orientations, several
            # seeds). Provably-infeasible targets are sampler optimism, not
            # planner faults — report SKIPPED_UNREACHABLE and exclude from the
            # rate gate. STRICT so it does not mask genuine PLAN_FAILs.
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
                        f"{episode} RESULT SKIPPED_UNREACHABLE "
                        f"{dex.as_log_tokens()} "
                        f"target=({target_xyz[0]:.3f},{target_xyz[1]:.3f},"
                        f"{target_xyz[2]:.3f}) | {_tally()}"
                    )
                    skipped_unreachable_log.append(result_line)
                    _viz_log(
                        f"  DEXTERITY_PRESCREEN: contact pose infeasible "
                        f"({dex.classification}; {dex.note}; region={dex.region_reason})"
                        " — excluded from rate gate, not a planner failure",
                        level="warn",
                    )
                    _viz_log(result_line, level="warn")
                    continue

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
                if contacted:
                    return
                pose = forward_kinematics(q_rad)
                ee = pose.position_m
                # Honest tip-face gate: reject "wrong side of the EE" (tool axis
                # not collinear with the approach) and "through the marker" (tip
                # crossed to the far hemisphere), in addition to lateral grazes.
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
                    contacted = True
                    _set_target_marker_color(stage, state=MarkerVisualState.CONTACT)
                    _viz_log(
                        "  MARKER_CONTACT: tip-face center on sphere → green "
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
                    label = {
                        "through": (
                            "MARKER_THROUGH: tip crossed to far hemisphere "
                            "(passed through marker) — not green"
                        ),
                        "side_graze": (
                            "MARKER_SIDE_GRAZE: contact off the tip-face pad "
                            "(lateral) — not green"
                        ),
                        "wrong_side_axis": (
                            "MARKER_WRONG_SIDE: flange +Z not aligned with the "
                            "outward approach ray — contact on the side/barrel "
                            "or flipped/back onto the marker, not the front "
                            "pad — not green"
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
                _viz_log(
                    f"  PLAN_FAIL backend={traj.backend} via_attempts={vias} "
                    f"msg={traj.message} (gate={gate}; marker=yellow after timeout)",
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
                    _on_step(q_hold)
                except Exception:
                    pass
            t_end = time.monotonic() + max(0.05, float(hold_s))
            while time.monotonic() < t_end and simulation_app.is_running():
                if not contacted:
                    try:
                        q_hold = _get_joint_positions(articulation)
                        _on_step(q_hold)
                    except Exception:
                        pass
                simulation_app.update()
            if not contacted:
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

                    ik = DampedLeastSquaresIK()
                    quat_now = forward_kinematics(q_now).quaternion_wxyz
                    res = ik.solve(
                        Pose(position_m=pierce, quaternion_wxyz=quat_now),
                        seed_q=q_now,
                    )
                    if bool(getattr(res, "success", False)):
                        _move_joints_at_hardware_speed(
                            articulation,
                            res.q,
                            simulation_app,
                            max_speed_rad_s=max_speed,
                            on_step=_on_step,
                        )
                    else:
                        _viz_log(
                            "  CONTACT_HOLD: axial IK did not converge "
                            f"({getattr(res, 'reason', 'unknown')})"
                        )
                except Exception as exc:  # noqa: BLE001
                    _viz_log(f"  CONTACT_HOLD: axial IK skipped ({exc})")
                t_nudge = time.monotonic() + max(0.15, float(hold_s))
                while time.monotonic() < t_nudge and simulation_app.is_running():
                    if not contacted:
                        try:
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
            if contacted:
                try:
                    p_fin = forward_kinematics(_get_joint_positions(articulation))
                    fok, freason, fm = classify_tip_contact(
                        np.asarray(p_fin.position_m, dtype=float).reshape(3),
                        target_xyz,
                        approach_from_m=approach_from_m,
                        ee_quaternion_wxyz=p_fin.quaternion_wxyz,
                    )
                    if not fok:
                        final_ok = False
                        n_invalid_side += 1
                        _set_target_marker_color(
                            stage, state=MarkerVisualState.PLAN_FAIL
                        )
                        _viz_log(
                            "  CONTACT_INVALID_SIDE: marker turned green during "
                            "motion, but the settled contact is invalid "
                            f"(reason={freason}, "
                            f"axis_in={np.degrees(fm['axis_in_err_rad']):.0f}deg "
                            f"axis_out={np.degrees(fm['axis_out_err_rad']):.0f}deg "
                            f"pen={fm['penetration_m'] * 1e3:.1f}mm "
                            f"lat={fm['lateral_m'] * 1e3:.1f}mm) — reported as "
                            "PLAN_FAIL despite the green flash",
                            level="error",
                        )
                except Exception:
                    pass

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
            else:
                # Reclassify: PLAN_OK without a valid settled contact is a failure.
                n_plan_ok -= 1
                n_plan_fail += 1
                _set_target_marker_color(stage, state=MarkerVisualState.PLAN_FAIL)
                if contacted:
                    # Invalid-side override (already logged CONTACT_INVALID_SIDE).
                    _viz_log(
                        f"{episode} RESULT PLAN_FAIL(invalid_side) | {_tally()}",
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

        metrics["phase2_plan_ok"] = n_plan_ok
        metrics["phase2_plan_fail"] = n_plan_fail
        metrics["phase2_marker_contact_green"] = n_contact_green
        metrics["phase2_via_waypoint_ok"] = n_via_ok
        metrics["phase2_skipped_targets"] = n_skip
        metrics["phase2_skipped_unreachable"] = n_skip_unreachable
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
        # Resolve min rate: CLI > env > YAML (0 disables the gate).
        if args.min_plan_ok_rate is not None:
            min_rate = float(args.min_plan_ok_rate)
        elif os.environ.get("ISAAC_VIZ_MIN_PLAN_OK_RATE", "").strip() != "":
            min_rate = float(os.environ["ISAAC_VIZ_MIN_PLAN_OK_RATE"])
        else:
            min_rate = float(plan_cfg.get("min_plan_ok_rate", 0.25))
        metrics["phase2_min_plan_ok_rate"] = min_rate
        write_json(json_out, metrics)
        _viz_log(
            f"Phase 2 planning summary: ok={n_plan_ok} fail={n_plan_fail} "
            f"via={n_via_ok} skip={n_skip} skip_unreachable={n_skip_unreachable} "
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
