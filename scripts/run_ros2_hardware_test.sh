#!/usr/bin/env bash
# ROS 2 hardware / dry-run helper.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${ENABLE_MYCOBOT_HARDWARE_TESTS:-}" == "1" ]]; then
  echo "WARNING: ENABLE_MYCOBOT_HARDWARE_TESTS=1 — hardware motion may be commanded."
else
  echo "Dry-run / validation path (hardware disabled)."
fi

cd "${ROOT}/ros2_ws"
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
colcon build --packages-select residual_adaptive_ik_ros --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
ros2 launch residual_adaptive_ik_ros residual_ik.launch.py mode:="${1:-validation_only}"
