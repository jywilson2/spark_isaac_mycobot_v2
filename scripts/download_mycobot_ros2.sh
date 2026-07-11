#!/usr/bin/env bash
# Obtain elephantrobotics/mycobot_ros2 for full URDF + meshes.
# Prefers a workspace sibling checkout, then clones into third_party/.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/third_party/mycobot_ros2"
URL="${MYCOBOT_ROS2_URL:-https://github.com/elephantrobotics/mycobot_ros2.git}"
SIBLING="${ROOT}/../mycobot_ros2"
URDF_REL="mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf"

mkdir -p "${ROOT}/third_party"

if [[ -L "${DEST}" ]]; then
  echo "Using symlink: ${DEST} -> $(readlink -f "${DEST}")"
elif [[ -f "${SIBLING}/${URDF_REL}" && ! -e "${DEST}" ]]; then
  ln -s "$(cd "${SIBLING}" && pwd)" "${DEST}"
  echo "Symlinked sibling checkout: ${DEST} -> ${SIBLING}"
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
  exit 1
else
  echo "Expected URDF missing: ${URDF}" >&2
  exit 1
fi
