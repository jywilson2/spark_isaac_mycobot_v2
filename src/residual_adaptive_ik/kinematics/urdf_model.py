# Copyright 2026 spark_isaac_mycobot_v2 contributors
"""Minimal URDF joint-chain loader for MyCobot 280 forward kinematics.

Uses only the Python standard library + NumPy so Phase 1 FK works in CI
without Pinocchio. Prefer ``third_party/mycobot_ros2`` when present; otherwise
fall back to the kinematics-only URDF under ``assets/urdf/``.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

# Revolute joints in base→EE order for mycobot_280_m5.urdf (maps to q[0]…q[5]).
DEFAULT_REVOLUTE_JOINT_NAMES: tuple[str, ...] = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)

DEFAULT_BASE_LINK = "g_base"
DEFAULT_EE_LINK = "joint6_flange"


def _repo_root() -> Path:
    env = os.environ.get("SPARK_REPO_ROOT")
    if env:
        return Path(env).resolve()
    # src/residual_adaptive_ik/kinematics/urdf_model.py → repo root
    return Path(__file__).resolve().parents[3]


def default_urdf_candidates() -> tuple[Path, ...]:
    root = _repo_root()
    return (
        root
        / "third_party"
        / "mycobot_ros2"
        / "mycobot_description"
        / "urdf"
        / "mycobot_280_m5"
        / "mycobot_280_m5.urdf",
        root / "assets" / "urdf" / "mycobot_280_m5_kinematics.urdf",
        # Workspace sibling checkout (DGX Spark isaac_ros-dev layout)
        root.parent / "mycobot_ros2" / "mycobot_description" / "urdf" / "mycobot_280_m5" / "mycobot_280_m5.urdf",
    )


def resolve_urdf_path(explicit: Path | str | None = None) -> Path:
    """Return the first readable MyCobot 280 M5 URDF path."""
    if explicit is not None:
        path = Path(explicit)
        if not path.is_file():
            raise FileNotFoundError(f"URDF not found: {path}")
        return path
    for candidate in default_urdf_candidates():
        if candidate.is_file():
            return candidate
    tried = "\n  ".join(str(p) for p in default_urdf_candidates())
    raise FileNotFoundError(
        "MyCobot 280 URDF not found. Run scripts/download_mycobot_ros2.sh or "
        f"ensure assets/urdf/mycobot_280_m5_kinematics.urdf exists.\nTried:\n  {tried}"
    )


def _parse_xyz(text: str | None) -> np.ndarray:
    if not text:
        return np.zeros(3, dtype=float)
    vals = [float(x) for x in text.replace(",", " ").split()]
    if len(vals) != 3:
        raise ValueError(f"expected 3 xyz values, got {text!r}")
    return np.asarray(vals, dtype=float)


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """Intrinsic XYZ fixed-axis RPY (URDF convention) → 3×3 rotation matrix."""
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _origin_to_matrix(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = _rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle_matrix(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n == 0.0:
        return np.eye(3, dtype=float)
    x, y, z = axis / n
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert 3×3 rotation matrix to unit quaternion ``(w, x, y, z)``."""
    m = np.asarray(R, dtype=float)
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    return q / np.linalg.norm(q)


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray  # 4×4
    axis: np.ndarray  # 3,


@dataclass(frozen=True)
class UrdfKinematicModel:
    """Serial chain from base link to EE link."""

    urdf_path: Path
    base_link: str
    ee_link: str
    chain: tuple[UrdfJoint, ...]
    revolute_names: tuple[str, ...]

    def _validate_q(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape != (len(self.revolute_names),):
            raise ValueError(
                f"q must have shape ({len(self.revolute_names)},), got {q.shape}"
            )
        if not np.all(np.isfinite(q)):
            raise ValueError("q must contain only finite values")
        return q

    def link_transforms(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Return 4×4 base←link transforms for ``base_link`` and each chain child.

        Used to place mesh-fitted collision spheres (centers stored in the
        **link frame**) into the robot base / world frame for debug viz and
        diagnostics. Units: meters / radians. See ``spec.md`` Phase 2.
        """
        q = self._validate_q(q)
        q_map = dict(zip(self.revolute_names, q, strict=True))
        out: dict[str, np.ndarray] = {self.base_link: np.eye(4, dtype=float)}
        T = np.eye(4, dtype=float)
        for joint in self.chain:
            T = T @ joint.origin
            if joint.joint_type == "revolute":
                angle = q_map[joint.name]
                R = _axis_angle_matrix(joint.axis, angle)
                T_motion = np.eye(4, dtype=float)
                T_motion[:3, :3] = R
                T = T @ T_motion
            elif joint.joint_type == "fixed":
                pass
            else:
                raise NotImplementedError(f"unsupported joint type: {joint.joint_type}")
            out[joint.child] = T.copy()
        return out

    def forward_transforms(
        self, q: np.ndarray
    ) -> tuple[np.ndarray, list[tuple[np.ndarray, np.ndarray]]]:
        """Return EE ``T_ee`` (4×4) and per-revolute ``(p_joint, z_axis)`` in base.

        ``p_joint`` / ``z_axis`` are taken at the joint axis after applying the
        joint origin, before the revolute motion — the geometric Jacobian frame.
        """
        q = self._validate_q(q)
        q_map = dict(zip(self.revolute_names, q, strict=True))
        T = np.eye(4, dtype=float)
        revolute_frames: list[tuple[np.ndarray, np.ndarray]] = []
        for joint in self.chain:
            T = T @ joint.origin
            if joint.joint_type == "revolute":
                p = T[:3, 3].copy()
                z = (T[:3, :3] @ joint.axis).copy()
                revolute_frames.append((p, z))
                angle = q_map[joint.name]
                R = _axis_angle_matrix(joint.axis, angle)
                T_motion = np.eye(4, dtype=float)
                T_motion[:3, :3] = R
                T = T @ T_motion
            elif joint.joint_type == "fixed":
                continue
            else:
                raise NotImplementedError(f"unsupported joint type: {joint.joint_type}")
        return T, revolute_frames

    def forward(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(position_m, quaternion_wxyz)`` for joint vector ``q`` (rad)."""
        T, _ = self.forward_transforms(q)
        return T[:3, 3].copy(), rotation_matrix_to_quaternion_wxyz(T[:3, :3])

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Geometric Jacobian ``J`` (6×n) mapping ``dq`` → ``[v; ω]`` in base frame.

        Units: linear part meters/radian, angular part dimensionless (rad/rad).
        """
        T_ee, frames = self.forward_transforms(q)
        p_ee = T_ee[:3, 3]
        n = len(self.revolute_names)
        J = np.zeros((6, n), dtype=float)
        for i, (p_i, z_i) in enumerate(frames):
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
        return J


def _parse_joints(root: ET.Element) -> dict[str, UrdfJoint]:
    joints: dict[str, UrdfJoint] = {}
    for elem in root.findall("joint"):
        name = elem.attrib["name"]
        jtype = elem.attrib["type"]
        parent = elem.find("parent").attrib["link"]  # type: ignore[union-attr]
        child = elem.find("child").attrib["link"]  # type: ignore[union-attr]
        origin_elem = elem.find("origin")
        xyz = _parse_xyz(origin_elem.attrib.get("xyz") if origin_elem is not None else None)
        rpy = _parse_xyz(origin_elem.attrib.get("rpy") if origin_elem is not None else None)
        axis_elem = elem.find("axis")
        axis = _parse_xyz(axis_elem.attrib.get("xyz") if axis_elem is not None else "0 0 1")
        joints[name] = UrdfJoint(
            name=name,
            joint_type=jtype,
            parent=parent,
            child=child,
            origin=_origin_to_matrix(xyz, rpy),
            axis=axis,
        )
    return joints


def _build_chain(
    joints_by_name: dict[str, UrdfJoint],
    *,
    base_link: str,
    ee_link: str,
) -> tuple[UrdfJoint, ...]:
    child_to_joint = {j.child: j for j in joints_by_name.values()}
    chain_rev: list[UrdfJoint] = []
    link = ee_link
    seen: set[str] = set()
    while link != base_link:
        if link in seen:
            raise ValueError(f"cycle while walking URDF to {base_link} from {ee_link}")
        seen.add(link)
        if link not in child_to_joint:
            raise ValueError(f"no parent joint for link {link!r} (base={base_link!r})")
        joint = child_to_joint[link]
        chain_rev.append(joint)
        link = joint.parent
    return tuple(reversed(chain_rev))


def load_urdf_model(
    urdf_path: Path | str | None = None,
    *,
    base_link: str = DEFAULT_BASE_LINK,
    ee_link: str = DEFAULT_EE_LINK,
    revolute_names: tuple[str, ...] = DEFAULT_REVOLUTE_JOINT_NAMES,
) -> UrdfKinematicModel:
    path = resolve_urdf_path(urdf_path)
    tree = ET.parse(path)
    root = tree.getroot()
    joints = _parse_joints(root)
    chain = _build_chain(joints, base_link=base_link, ee_link=ee_link)
    for name in revolute_names:
        if name not in joints:
            raise KeyError(f"revolute joint {name!r} missing from {path}")
        if joints[name].joint_type != "revolute":
            raise ValueError(f"joint {name!r} is not revolute in {path}")
    return UrdfKinematicModel(
        urdf_path=path,
        base_link=base_link,
        ee_link=ee_link,
        chain=chain,
        revolute_names=revolute_names,
    )


@lru_cache(maxsize=4)
def get_default_model() -> UrdfKinematicModel:
    return load_urdf_model()
