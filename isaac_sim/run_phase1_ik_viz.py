#!/usr/bin/env python3
# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Phase 1 classical IK metrics **with** Isaac Sim visualization (host).

One run:
  1. Evaluates ``--num-poses`` DLS IK trials (Phase 1 metrics).
  2. Writes JSON + Markdown reports.
  3. Animates a representative subset; each goal is a small sphere that stays
     **red until EE contact**, then turns **green** (meters; see
     ``isaac_sim/target_marker.py``).

Run on the **host**:

    ./scripts/host/run_phase1_isaac.sh
    # quick test:
    ./scripts/host/run_phase1_isaac.sh --skip-tests -- \\
        --num-poses 240 --visualize 48 --hold-s 0.4

See ``spec.md`` Phase 1.
"""
from __future__ import annotations

import argparse
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
    TARGET_MARKER_COLOR_GREEN_RGB,
    TARGET_MARKER_COLOR_RED_RGB,
    TARGET_MARKER_CONTACT_DISTANCE_M,
    TARGET_MARKER_EMISSIVE_GREEN_RGB,
    TARGET_MARKER_EMISSIVE_RED_RGB,
    TARGET_MARKER_RADIUS_M,
    ee_contacts_target,
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
from residual_adaptive_ik.utils.logging_utils import write_json  # noqa: E402
from residual_adaptive_ik.kinematics.workspace_sampling import (  # noqa: E402
    load_workspace_config,
)

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
        help="Seconds to hold each visualized IK solution",
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
    return parser.parse_args()


def _marker_rgb(contacted: bool) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (diffuse, emissive) RGB for red (pre-contact) or green (contact)."""
    if contacted:
        return TARGET_MARKER_COLOR_GREEN_RGB, TARGET_MARKER_EMISSIVE_GREEN_RGB
    return TARGET_MARKER_COLOR_RED_RGB, TARGET_MARKER_EMISSIVE_RED_RGB


def _set_target_marker_color(stage, *, contacted: bool) -> None:
    """Update existing IkTarget material color (red ↔ green).

    Color is applied only via UsdPreviewSurface — do **not** set
    ``primvars:displayColor`` without indices (Fabric warns otherwise).
    """
    from pxr import Gf, Sdf, UsdShade  # noqa: WPS433

    color_rgb, emit_rgb = _marker_rgb(contacted)
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
    contacted: bool = False,
    radius_m: float = TARGET_MARKER_RADIUS_M,
) -> None:
    """Place a small sphere at the IK target (meters).

    Starts **red** (``contacted=False``). Call ``_set_target_marker_color`` when
    the EE tip enters ``TARGET_MARKER_CONTACT_DISTANCE_M`` of the goal center.

    The marker is a **visual-only** prim (no PhysX collision). Phase 1 IK does
    not plan around it; links may sweep through the sphere in joint space.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: WPS433

    path = Sdf.Path("/World/IkTarget")
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.GetRadiusAttr().Set(float(radius_m))
    color_rgb, emit_rgb = _marker_rgb(contacted)
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
        gxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
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
            "`isaac_sim/run_phase1_ik_viz.py` / `scripts/host/run_phase1_isaac.sh`"
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

        print(
            f"Visualizing {len(to_show)}/{len(trials)} trials "
            f"(hold={args.hold_s}s; sphere red→green on EE contact "
            f"≤{TARGET_MARKER_CONTACT_DISTANCE_M * 1e3:.0f} mm; "
            f"servo≤{workspace.max_joint_speed_deg_s:g}°/s)..."
        )
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(10):
            simulation_app.update()

        articulation = _create_articulation(prim_path)
        max_speed = float(workspace.max_joint_speed_rad_s)
        for i, trial in enumerate(to_show):
            status = "OK" if trial.success else f"FAIL({trial.reason})"
            print(
                f"[{i + 1}/{len(to_show)}] trial={trial.index} {status} "
                f"pos_err_m={trial.position_error_m:.4e} "
                f"xyz=({trial.target.position_m[0]:.3f},"
                f"{trial.target.position_m[1]:.3f},"
                f"{trial.target.position_m[2]:.3f})"
            )
            target_xyz = np.asarray(trial.target.position_m, dtype=float).reshape(3)
            _set_target_marker(stage, target_xyz, contacted=False)
            contacted = False

            def _on_step(q_rad: np.ndarray, *, _tgt=target_xyz) -> None:
                nonlocal contacted
                if contacted:
                    return
                ee = forward_kinematics(q_rad).position_m
                if ee_contacts_target(ee, _tgt):
                    contacted = True
                    _set_target_marker_color(stage, contacted=True)

            _move_joints_at_hardware_speed(
                articulation,
                trial.q_sol,
                simulation_app,
                max_speed_rad_s=max_speed,
                on_step=_on_step,
            )
            t_end = time.monotonic() + max(0.05, float(args.hold_s))
            while time.monotonic() < t_end and simulation_app.is_running():
                if not contacted:
                    try:
                        q_now = _get_joint_positions(articulation)
                        _on_step(q_now)
                    except Exception:
                        pass
                simulation_app.update()
            if not simulation_app.is_running():
                break

        print("Phase 1 metrics + visualization complete. Close Isaac Sim or Ctrl+C.")
        if not args.headless and not args.auto_exit:
            while simulation_app.is_running():
                simulation_app.update()
        elif not args.headless and args.auto_exit:
            print("NOTE: --auto-exit set; closing Kit after visualization (agent/GUI smoke).")
        return 0
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
            "  ./scripts/host/run_phase1_isaac.sh\n"
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
