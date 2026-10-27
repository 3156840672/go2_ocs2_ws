#!/usr/bin/env bash
# Source from each native Humble terminal.
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "请打开未加载其他ROS发行版的新终端，再source本文件。" >&2
  return 1
fi
GO2_VERIFIED_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_REPOSITORY_ROOT="$(cd "$GO2_VERIFIED_WS/.." && pwd)"
source /opt/ros/humble/setup.bash

# Prefer the reproducible in-repository dependency workspace. An existing OCS2
# installation can be selected explicitly with OCS2_UNDERLAY=/path/to/install.
if [[ -z "${OCS2_UNDERLAY:-}" ]]; then
  if [[ -f "$GO2_REPOSITORY_ROOT/.dependencies/ocs2_ws/install/local_setup.bash" ]]; then
    OCS2_UNDERLAY="$GO2_REPOSITORY_ROOT/.dependencies/ocs2_ws/install"
  elif [[ -f "$HOME/ros2_ws/install/local_setup.bash" ]]; then
    OCS2_UNDERLAY="$HOME/ros2_ws/install"
  fi
fi
if [[ -n "${OCS2_UNDERLAY:-}" && -f "$OCS2_UNDERLAY/local_setup.bash" ]]; then
  source "$OCS2_UNDERLAY/local_setup.bash"
else
  echo "未找到OCS2 underlay。首次使用请在仓库根目录运行 scripts/bootstrap_humble.sh。" >&2
fi
if [[ -f "$GO2_VERIFIED_WS/install/local_setup.bash" ]]; then
  source "$GO2_VERIFIED_WS/install/local_setup.bash"
fi

if [[ -z "${MUJOCO_DIR:-}" ]]; then
  for candidate in /usr/local "$HOME/.mujoco/mujoco-3.3.0" "$HOME/.local/mujoco/mujoco-3.3.0"; do
    if [[ -f "$candidate/include/mujoco/mujoco.h" && -f "$candidate/lib/libmujoco.so" ]]; then
      MUJOCO_DIR="$candidate"
      break
    fi
  done
fi
export MUJOCO_DIR="${MUJOCO_DIR:-/usr/local}"
export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:${LD_LIBRARY_PATH:-}"
export GO2_CPPAD_MODEL_DIR="${GO2_CPPAD_MODEL_DIR:-$GO2_REPOSITORY_ROOT/.dependencies/cppad/go2}"
export OPENBLAS_NUM_THREADS=1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${GO2_ROS_DOMAIN_ID:-42}"
