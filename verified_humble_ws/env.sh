#!/usr/bin/env bash
# Source from each native Humble terminal.
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "请打开未加载其他ROS发行版的新终端，再source本文件。" >&2
  return 1
fi
GO2_VERIFIED_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
# The verified Humble OCS2 stack (ocs2_core, ocs2_sqp, Pinocchio wrappers,
# terrain dependencies) is installed in this local underlay.
if [[ -f /home/yue/ros2_ws/install/local_setup.bash ]]; then
  source /home/yue/ros2_ws/install/local_setup.bash
fi
if [[ -f "$GO2_VERIFIED_WS/install/local_setup.bash" ]]; then
  source "$GO2_VERIFIED_WS/install/local_setup.bash"
fi
export MUJOCO_DIR="${MUJOCO_DIR:-/usr/local}"
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${GO2_ROS_DOMAIN_ID:-42}"
