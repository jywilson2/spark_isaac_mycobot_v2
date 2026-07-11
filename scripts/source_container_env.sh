#!/usr/bin/env bash
# Source from Isaac ROS container terminals when ~/.bashrc is bind-mounted read-only.
#   source /workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2/scripts/source_container_env.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export SPARK_REPO_ROOT="${SPARK_REPO_ROOT:-/workspaces/isaac_ros-dev/src/spark_isaac_mycobot_v2}"
export PYTHONPATH="${SPARK_REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
