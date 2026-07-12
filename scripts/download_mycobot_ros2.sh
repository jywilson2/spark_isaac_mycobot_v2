#!/usr/bin/env bash
# Obtain elephantrobotics/mycobot_ros2 for full URDF + meshes.
# Prefers a workspace sibling checkout, then clones into third_party/.
#
# Symlinks use a *relative* target (../../mycobot_ros2) so the same repo works
# in the Isaac ROS container (/workspaces/...) and on the DGX Spark host
# (/home/.../workspaces/...) without rewriting absolute paths.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/third_party/mycobot_ros2"
URL="${MYCOBOT_ROS2_URL:-https://github.com/elephantrobotics/mycobot_ros2.git}"
SIBLING="$(cd "${ROOT}/.." && pwd)/mycobot_ros2"
URDF_REL="mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf"
# Relative from third_party/mycobot_ros2 → ../../mycobot_ros2
REL_LINK_TARGET="../../mycobot_ros2"

mkdir -p "${ROOT}/third_party"

_urdf_readable() {
  [[ -f "${DEST}/${URDF_REL}" ]]
}

_link_sibling() {
  # Remove broken or absolute-/workspaces symlinks that fail on the host.
  if [[ -L "${DEST}" ]] || [[ -e "${DEST}" ]]; then
    rm -rf "${DEST}"
  fi
  ln -s "${REL_LINK_TARGET}" "${DEST}"
  echo "Symlinked sibling checkout: ${DEST} -> ${REL_LINK_TARGET} (resolves to ${SIBLING})"
}

if [[ -f "${SIBLING}/${URDF_REL}" ]]; then
  need_relink=0
  if ! _urdf_readable; then
    need_relink=1
  elif [[ -L "${DEST}" ]]; then
    current="$(readlink "${DEST}")"
    # Absolute /workspaces/... links break on the host mount namespace.
    if [[ "${current}" == /* ]]; then
      echo "Replacing absolute vendor symlink (${current}) with relative ${REL_LINK_TARGET}"
      need_relink=1
    fi
  fi
  if [[ "${need_relink}" -eq 1 ]]; then
    _link_sibling
  elif [[ -L "${DEST}" ]]; then
    echo "Using symlink: ${DEST} -> $(readlink "${DEST}") (ok: ${DEST}/${URDF_REL})"
  else
    echo "Using existing ${DEST}"
  fi
elif [[ -L "${DEST}" ]]; then
  echo "WARNING: ${DEST} is a symlink but sibling URDF not found at ${SIBLING}/${URDF_REL}" >&2
elif [[ -d "${DEST}/.git" ]]; then
  git -C "${DEST}" fetch --depth 1 origin || true
  git -C "${DEST}" pull --ff-only || true
elif [[ -d "${DEST}" ]]; then
  echo "Using existing ${DEST}"
else
  git clone --depth 1 "${URL}" "${DEST}"
fi

URDF="${DEST}/${URDF_REL}"
ASSET_URDF="${ROOT}/assets/urdf/mycobot_280_m5_kinematics.urdf"
if [[ -f "${URDF}" ]]; then
  echo "mycobot_ros2 ready: ${URDF}"
elif [[ -f "${ASSET_URDF}" ]]; then
  echo "Vendor URDF missing at ${URDF}; kinematics-only asset still available: ${ASSET_URDF}" >&2
  echo "Rendered Isaac Sim import needs vendor meshes — fix sibling checkout or clone." >&2
  exit 1
else
  echo "Expected URDF missing: ${URDF}" >&2
  exit 1
fi
