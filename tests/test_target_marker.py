# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Target-marker constants and contact coloring (no Kit required)."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from isaac_sim.target_marker import (
    TARGET_MARKER_COLOR_GREEN_RGB,
    TARGET_MARKER_COLOR_RED_RGB,
    TARGET_MARKER_COLOR_YELLOW_RGB,
    TARGET_MARKER_RADIUS_M,
    TARGET_MARKER_TIP_FACE_RADIUS_M,
    ee_contacts_target,
    marker_rgb_for_state,
    tip_face_pierce_point_m,
)
from isaac_sim.viz_plan_policy import MarkerVisualState


REPO = Path(__file__).resolve().parents[1]


def test_target_marker_radius_matches_v1():
    """v1 Isaac Lab uses a 12 mm sphere; Phase 1 viz must match size."""
    assert TARGET_MARKER_RADIUS_M == 0.012
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert "UsdGeom.Sphere" in src
    assert "ee_contacts_target" in src
    assert "TARGET_MARKER_COLOR_GREEN_RGB" in src or "contacted" in src
    assert "approach_from_m" in src
    assert "tip-face" in src.lower() or "tip_face" in src


def test_ee_contacts_target_surface_shell():
    """Without approach ray: tip must sit in the surface shell (not immersed)."""
    from isaac_sim.target_marker import TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M

    target = np.array([0.20, 0.0, 0.15])
    far = target + np.array([0.05, 0.0, 0.0])
    assert not ee_contacts_target(far, target)
    # On the surface (radius).
    on_surface = target + np.array([TARGET_MARKER_RADIUS_M, 0.0, 0.0])
    assert ee_contacts_target(on_surface, target)
    # Deep inside the volume is immersion — not contact.
    inside = target + np.array([TARGET_MARKER_RADIUS_M * 0.5, 0.0, 0.0])
    assert not ee_contacts_target(inside, target)
    # Just outside radius but within outer tolerance → still contact.
    just_out = target + np.array(
        [TARGET_MARKER_RADIUS_M + TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M * 0.5, 0.0, 0.0]
    )
    assert ee_contacts_target(just_out, target)
    beyond = target + np.array(
        [TARGET_MARKER_RADIUS_M + TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M + 0.001, 0.0, 0.0]
    )
    assert not ee_contacts_target(beyond, target)


def test_ee_contacts_target_requires_tip_face_not_side():
    """With approach ray: only tip-face center counts — side grazes are invalid."""
    target = np.array([0.20, 0.0, 0.15])
    # Approach along +X: start west of the marker.
    approach_from = target + np.array([-0.10, 0.0, 0.0])
    pierce = tip_face_pierce_point_m(approach_from, target)
    assert float(np.linalg.norm(pierce - target)) == pytest.approx(
        TARGET_MARKER_RADIUS_M, abs=1e-9
    )
    assert ee_contacts_target(
        pierce, target, approach_from_m=approach_from
    )
    # Slight offset still within tip-face pad (and still on the surface shell).
    near_pad = pierce + np.array([0.0, TARGET_MARKER_TIP_FACE_RADIUS_M * 0.5, 0.0])
    # Keep near_pad on the surface radius so immersion does not reject it.
    near_pad = target + (TARGET_MARKER_RADIUS_M / float(np.linalg.norm(near_pad - target))) * (
        near_pad - target
    )
    assert ee_contacts_target(
        near_pad, target, approach_from_m=approach_from
    )
    # Axial immersion into the volume must NOT count (surface touch required).
    inside_axis = target + np.array([-TARGET_MARKER_RADIUS_M * 0.4, 0.0, 0.0])
    assert not ee_contacts_target(
        inside_axis, target, approach_from_m=approach_from
    )
    # Equator / side graze: on the sphere but far from pierce → not contact.
    side = target + np.array([0.0, TARGET_MARKER_RADIUS_M, 0.0])
    assert float(np.linalg.norm(side - target)) == pytest.approx(
        TARGET_MARKER_RADIUS_M, abs=1e-9
    )
    assert not ee_contacts_target(
        side, target, approach_from_m=approach_from
    )


def test_ee_contacts_target_requires_tool_axis_alignment():
    """Flange +Z must point OUTWARD (along the approach ray) — signed check.

    A genuine front tip-face contact has +Z along the outward normal
    (tip − center); this is the reachable, visually-correct orientation
    (commanding the inward sign fails IK). The gate is *signed*: it rejects both
    - "side of the EE": tool axis ⟂ the radial (+Z ⊥ approach), and
    - "flipped/back of the EE": +Z pointing *toward* the center.
    """
    from residual_adaptive_ik.planning.contact_geometry import (
        quaternion_facing_sphere,
    )

    target = np.array([0.20, 0.0, 0.15])
    approach_from = target + np.array([-0.10, 0.0, 0.0])
    pierce = tip_face_pierce_point_m(approach_from, target)
    # +Z outward (away from center, along approach ray): correct front contact.
    quat_out = quaternion_facing_sphere(pierce - target)
    assert ee_contacts_target(
        pierce, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quat_out, axis_tol_rad=0.2,
    )
    # +Z inward (toward center): flange flipped/back onto the marker even though
    # the tip position is correct — reject.
    quat_in = quaternion_facing_sphere(target - pierce)
    assert not ee_contacts_target(
        pierce, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quat_in, axis_tol_rad=0.2,
    )
    # Perpendicular (tool axis along world +Z, ⟂ the +X approach): sphere would
    # touch the *side* of the EE — reject ("wrong side of the EE").
    quat_side = quaternion_facing_sphere(np.array([0.0, 0.0, 1.0]))
    assert not ee_contacts_target(
        pierce, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quat_side, axis_tol_rad=0.2,
    )


def test_classify_tip_contact_detects_failure_modes():
    """classify_tip_contact labels through / side / wrong-side / ok."""
    from isaac_sim.target_marker import classify_tip_contact
    from residual_adaptive_ik.planning.contact_geometry import (
        quaternion_facing_sphere,
    )

    target = np.array([0.20, 0.0, 0.15])
    approach_from = target + np.array([-0.10, 0.0, 0.0])  # approach along +X
    pierce = tip_face_pierce_point_m(approach_from, target)  # on -X surface
    normal = pierce - target  # outward at pierce (−X) — correct +Z direction
    inward = target - pierce  # toward center (+X)

    # Valid front tip-face contact: +Z outward (along the approach ray).
    ok, reason, m = classify_tip_contact(
        pierce, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(normal),
    )
    assert ok and reason == "ok"
    # penetration ≈ −radius on the near surface (signed axial from center).
    assert m["penetration_m"] == pytest.approx(-TARGET_MARKER_RADIUS_M, abs=2e-3)
    # axis_out ≈ 0 (flange faces back along approach); axis_in ≈ π.
    assert m["axis_out_err_rad"] == pytest.approx(0.0, abs=1e-6)
    assert m["axis_in_err_rad"] == pytest.approx(np.pi, abs=1e-6)

    # Through: tip on the far (+X) *surface* along the approach (dist≈radius so
    # the surface-shell check does not classify it as immersed first).
    far_side = target + np.array([TARGET_MARKER_RADIUS_M, 0.0, 0.0])
    ok, reason, m = classify_tip_contact(
        far_side, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(normal),
    )
    assert not ok and reason == "through"
    assert m["penetration_m"] > 0.0

    # Side graze: on the equator, far from the tip-face pad.
    side = target + np.array([0.0, TARGET_MARKER_RADIUS_M, 0.0])
    ok, reason, _ = classify_tip_contact(
        side, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(side - target),
    )
    assert not ok and reason == "side_graze"

    # Wrong side of the EE (side/barrel): correct position but tool axis ⟂ radial.
    ok, reason, _ = classify_tip_contact(
        pierce, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(np.array([0.0, 0.0, 1.0])),
    )
    assert not ok and reason == "wrong_side_axis"

    # Flipped/back onto the marker: correct position but +Z points TOWARD the
    # center (inward). This is the case the sign-agnostic gate used to accept
    # (folded 180°→0°) — now rejected by the signed outward check.
    ok, reason, m = classify_tip_contact(
        pierce, target, approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(inward),
    )
    assert not ok and reason == "wrong_side_axis"
    assert m["axis_out_err_rad"] == pytest.approx(np.pi, abs=1e-6)

    # Too far away entirely.
    ok, reason, _ = classify_tip_contact(
        target + np.array([0.05, 0.0, 0.0]), target,
        approach_from_m=approach_from,
    )
    assert not ok and reason == "no_contact"


def test_classify_tip_contact_rejects_immersion():
    """Tip deep inside the sphere volume is immersed, not surface contact."""
    from isaac_sim.target_marker import classify_tip_contact

    target = np.array([0.20, 0.0, 0.15])
    approach_from = target + np.array([-0.10, 0.0, 0.0])
    deep = target + np.array([-TARGET_MARKER_RADIUS_M * 0.3, 0.0, 0.0])
    ok, reason, m = classify_tip_contact(
        deep, target, approach_from_m=approach_from
    )
    assert not ok and reason == "immersed"
    assert m["dist_m"] < TARGET_MARKER_RADIUS_M


def test_classify_tip_contact_rejects_prior_false_green_metrics():
    """Last-smoke false greens (axis≈25°, lat≈8 mm, dist≈15 mm) must fail.

    Those samples previously counted as PLAN_OK under the loose ≈35° / 10 mm /
    4 mm outer gate. Honest tip-face greens are axis_out ≲ 15°, small lateral,
    and tip within 2 mm of the surface.
    """
    from isaac_sim.target_marker import (
        TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M,
        TARGET_MARKER_TIP_FACE_RADIUS_M,
        TARGET_MARKER_TOOL_AXIS_TOL_RAD,
        classify_tip_contact,
    )
    from residual_adaptive_ik.planning.contact_geometry import (
        quaternion_facing_sphere,
    )

    assert TARGET_MARKER_TOOL_AXIS_TOL_RAD == pytest.approx(0.26, abs=1e-6)
    assert TARGET_MARKER_TIP_FACE_RADIUS_M == pytest.approx(0.004, abs=1e-9)
    assert TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M == pytest.approx(0.002, abs=1e-9)

    target = np.array([0.20, 0.0, 0.15])
    approach_from = target + np.array([-0.10, 0.0, 0.0])
    pierce = tip_face_pierce_point_m(approach_from, target)
    outward = pierce - target

    # Ep1-like: tip short of surface (~15.2 mm from center) + large lateral.
    short_and_wide = target + np.array([-0.0152, 0.0081, 0.0])
    ok, reason, m = classify_tip_contact(
        short_and_wide,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(outward),
    )
    assert ok is False
    assert reason in ("no_contact", "side_graze")
    assert m["dist_m"] > TARGET_MARKER_RADIUS_M + TARGET_MARKER_SURFACE_CONTACT_OUTER_TOL_M - 1e-9 or (
        m["lateral_m"] > TARGET_MARKER_TIP_FACE_RADIUS_M
    )

    # Axis_out ≈ 25° on an otherwise near-pierce tip — reject under 15° tol.
    # Rotate outward about world +Y by 25°.
    ang = np.deg2rad(25.0)
    c, s = np.cos(ang), np.sin(ang)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)
    tilted = Ry @ (outward / float(np.linalg.norm(outward)))
    ok, reason, m = classify_tip_contact(
        pierce,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(tilted),
    )
    assert ok is False
    assert reason == "wrong_side_axis"
    assert m["axis_out_err_rad"] == pytest.approx(ang, abs=1e-3)
    assert m["axis_out_err_rad"] > TARGET_MARKER_TOOL_AXIS_TOL_RAD

    # Lateral ≈ 8 mm on the surface shell — exceeds 4 mm tip-face pad.
    wide = pierce + np.array([0.0, 0.008, 0.0])
    wide = target + (TARGET_MARKER_RADIUS_M / float(np.linalg.norm(wide - target))) * (
        wide - target
    )
    ok, reason, m = classify_tip_contact(
        wide,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(outward),
    )
    assert ok is False
    assert reason == "side_graze"
    assert m["lateral_m"] > TARGET_MARKER_TIP_FACE_RADIUS_M


def test_rejects_side_barrel_back_and_submerged_ee_contacts():
    """Gates must *fail* (not green) for the operator-visible bad approaches.

    Covers:
    * marker on the EE **side/barrel** (tool +Z ⟂ radial),
    * marker on the EE **back** (flange flipped, +Z toward center),
    * tip **submerged** into the sphere volume (immersed),
    * tip on the equator far from the tip-face pad (side_graze).

    If any of these returned ``ok``, the suite would incorrectly accept
    visually wrong contacts (the defect these gates exist to catch).
    """
    from isaac_sim.target_marker import classify_tip_contact
    from residual_adaptive_ik.planning.contact_geometry import (
        quaternion_facing_sphere,
    )

    target = np.array([0.20, 0.0, 0.15])
    approach_from = target + np.array([-0.10, 0.0, 0.0])
    pierce = tip_face_pierce_point_m(approach_from, target)
    outward = pierce - target
    inward = -outward

    # Side/barrel of EE: tip on pierce but tool axis world-+Z (⟂ approach).
    ok, reason, m = classify_tip_contact(
        pierce,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(np.array([0.0, 0.0, 1.0])),
    )
    assert ok is False
    assert reason == "wrong_side_axis"
    assert m["axis_out_err_rad"] > np.deg2rad(15.0)

    # Back of EE: tip on pierce but +Z toward sphere center.
    ok, reason, m = classify_tip_contact(
        pierce,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(inward),
    )
    assert ok is False
    assert reason == "wrong_side_axis"
    assert m["axis_out_err_rad"] == pytest.approx(np.pi, abs=1e-6)

    # Submerged EE / tip into volume (surface of EE immersed in target).
    submerged = target + outward / float(np.linalg.norm(outward)) * (
        TARGET_MARKER_RADIUS_M * 0.25
    )
    ok, reason, m = classify_tip_contact(
        submerged,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(outward),
    )
    assert ok is False
    assert reason == "immersed"
    assert m["dist_m"] < TARGET_MARKER_RADIUS_M

    # Equator graze: on surface but off the tip-face pad.
    side = target + np.array([0.0, TARGET_MARKER_RADIUS_M, 0.0])
    ok, reason, m = classify_tip_contact(
        side,
        target,
        approach_from_m=approach_from,
        ee_quaternion_wxyz=quaternion_facing_sphere(side - target),
    )
    assert ok is False
    assert reason == "side_graze"
    assert m["lateral_m"] > TARGET_MARKER_TIP_FACE_RADIUS_M


def test_red_and_green_rgb_distinct():
    assert TARGET_MARKER_COLOR_RED_RGB[0] > TARGET_MARKER_COLOR_RED_RGB[1]
    assert TARGET_MARKER_COLOR_GREEN_RGB[1] > TARGET_MARKER_COLOR_GREEN_RGB[0]


def test_yellow_rgb_for_plan_fail():
    assert TARGET_MARKER_COLOR_YELLOW_RGB[0] > 0.8
    assert TARGET_MARKER_COLOR_YELLOW_RGB[1] > 0.7
    assert TARGET_MARKER_COLOR_YELLOW_RGB[2] < 0.3
    assert marker_rgb_for_state(MarkerVisualState.PLAN_FAIL)[0] == TARGET_MARKER_COLOR_YELLOW_RGB


def test_target_marker_avoids_display_color_without_indices():
    """Fabric warns if displayColor is set without primvars:displayColor:indices."""
    src = (REPO / "isaac_sim" / "run_ik_viz.py").read_text(encoding="utf-8")
    assert ".CreateDisplayColorAttr(" not in src
    assert "UsdPreviewSurface" in src
    # Phase 2: marker is a volumetric planning obstacle (not visual-only).
    assert "volumetric" in src
    assert "obstacles=[target_obstacle]" in src or "SphereObstacle" in src
    assert "MarkerVisualState.PLAN_FAIL" in src
