#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPS_WS="${GO2_DEPS_WS:-$ROOT/.dependencies/ocs2_ws}"
OCS2_SRC="$DEPS_WS/src/ocs2_ros2"
OCS2_REV="cd23b0c3fd131aa3eed12d77371c7ac9a5a38828"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "缺少ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
  exit 1
fi
for command in git colcon rosdep; do
  command -v "$command" >/dev/null || {
    echo "缺少命令：$command" >&2
    exit 1
  }
done

source /opt/ros/humble/setup.bash
mkdir -p "$DEPS_WS/src"
if [[ ! -d "$OCS2_SRC/.git" ]]; then
  git clone https://github.com/legubiao/ocs2_ros2.git "$OCS2_SRC"
fi
git -C "$OCS2_SRC" fetch origin "$OCS2_REV"
git -C "$OCS2_SRC" checkout --detach "$OCS2_REV"
git -C "$OCS2_SRC" submodule update --init --recursive

OCS2_PATHS=(
  "$OCS2_SRC/core"
  "$OCS2_SRC/mpc"
  "$OCS2_SRC/robotics"
  "$OCS2_SRC/basic examples/ocs2_legged_robot"
  "$OCS2_SRC/basic examples/ocs2_legged_robot_ros"
  "$OCS2_SRC/submodules/ocs2_robotic_assets"
  "$OCS2_SRC/submodules/plane_segmentation_ros2"
)

if [[ "${SKIP_ROSDEP:-0}" != "1" ]]; then
  rosdep install --from-paths "${OCS2_PATHS[@]}" --ignore-src -r -y
  rosdep install --from-paths "$ROOT/verified_humble_ws/src" --ignore-src -r -y \
    --skip-keys="qpoases_colcon ocs2_self_collision ocs2_legged_robot_ros ocs2_sphere_approximation convex_plane_decomposition_msgs convex_plane_decomposition_ros"
fi

colcon --log-base "$DEPS_WS/log" build \
  --base-paths "${OCS2_PATHS[@]}" \
  --build-base "$DEPS_WS/build" \
  --install-base "$DEPS_WS/install" \
  --packages-up-to ocs2_legged_robot_ros ocs2_sphere_approximation convex_plane_decomposition_ros \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

export OCS2_UNDERLAY="$DEPS_WS/install"
source "$ROOT/verified_humble_ws/env.sh"
colcon --log-base "$ROOT/verified_humble_ws/log" build \
  --base-paths "$ROOT/verified_humble_ws/src" \
  --build-base "$ROOT/verified_humble_ws/build" \
  --install-base "$ROOT/verified_humble_ws/install" \
  --packages-up-to joystick_input ocs2_quadruped_controller verified_go2_mujoco \
  --symlink-install \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "依赖和项目编译完成。"
echo "下一步：cd '$ROOT/verified_humble_ws' && source env.sh"
