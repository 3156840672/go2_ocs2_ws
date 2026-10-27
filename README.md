# Go2 OCS2 Controller for ROS 2 Humble

本项目在 Ubuntu 22.04 / ROS 2 Humble 上运行 Go2 四足机器人仿真，控制链全部使用 C++：

```text
键盘或手柄 -> OCS2 SQP-NMPC -> Weighted WBC
             -> q_des, dq_des, tau_ff, kp, kd -> MuJoCo
             -> 关节、IMU、接触力 -> 线性卡尔曼状态估计
```

默认使用仓库内的 Go2 MJCF 模型和进程内 `mujoco_ros2_control` 后端，不依赖 Docker、Unitree DDS 或外部仿真工程。

## 已验证功能

- 固定站立、trot、flying trot、pace、dynamic walk 和 static walk 步态。
- 键盘控制以及 Xbox 风格手柄输入；雷神 G30S TE 已完成实际映射验证。
- OCS2 SQP-MPC、Weighted WBC、线性卡尔曼状态估计和四足接触检测。
- MuJoCo 图形/无头仿真、完整 Go2 RViz 模型、预测轨迹、足端力与未来落足点。
- `odom -> base` 使用控制器估计结果；MuJoCo 真值单独发布在 `/go2/base_odom`。

## 系统要求

- Ubuntu 22.04
- ROS 2 Humble Desktop
- C++17 编译器、`colcon`、`rosdep` 和 Git
- MuJoCo 3.3.x 与 GLFW
- 首次准备依赖和生成 CppAD 动态库时需要网络

安装 ROS 工具和 GLFW：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libglfw3-dev ros-humble-desktop ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-joy
```

从 MuJoCo 官方发布包安装 3.3.x 后设置安装目录，例如：

```bash
export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.3.0"
```

`env.sh`也会自动检查 `/usr/local`、`~/.mujoco/mujoco-3.3.0` 和 `~/.local/mujoco/mujoco-3.3.0`。

## 首次安装

克隆仓库后，进入仓库根目录运行：

```bash
cd go2_ocs2_ws
./scripts/bootstrap_humble.sh
```

脚本会完成以下操作：

1. 将 `legubiao/ocs2_ros2` 固定到已验证提交 `cd23b0c3`。
2. 初始化 OCS2 子模块并安装可由 rosdep 解析的依赖。
3. 在 `.dependencies/ocs2_ws` 编译 OCS2 underlay。
4. 从固定提交构建 qpOASES，并编译本项目的 ROS 2 包。

已有独立 OCS2 工作空间时，可以跳过依赖构建并在终端中指定：

```bash
export OCS2_UNDERLAY=/path/to/ocs2_ws/install
cd verified_humble_ws
source env.sh
colcon build --base-paths src --packages-up-to \
  joystick_input ocs2_quadruped_controller verified_go2_mujoco \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 首次生成 CppAD 库

CppAD 生成库不提交到 Git。第一次安装或修改 URDF/动力学后执行：

```bash
cd verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py \
  headless:=true rviz:=false recompile_cppad:=true
```

看到 `MRT initialized. MPC thread started.` 后按 `Ctrl+C`。生成结果默认位于仓库的 `.dependencies/cppad/go2`，该目录不会提交到 Git；可通过 `GO2_CPPAD_MODEL_DIR` 修改位置。之后正常启动时不要再设置 `recompile_cppad:=true`。

## 启动

终端 A：

```bash
cd verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true
```

不使用手柄时，在终端 B 启动键盘：

```bash
cd verified_humble_ws
source env.sh
ros2 run keyboard_input keyboard_input
```

机器人初始处于 passive，会自然趴下。控制器加载完成后按 `2` 进入 stance，按 `3` 进入 trot，使用 `W/A/S/D/J/L` 调节运动，空格清零速度，最后按 `2` 回到 stance。

## 文档

- [USAGE.md](USAGE.md)：启动方式、键盘/手柄映射和故障排查。
- [PROJECT_GUIDE.md](PROJECT_GUIDE.md)：控制流程、状态估计、OCS2 和 WBC 原理。
- [dependencies.repos](dependencies.repos)：经过验证的外部 OCS2 版本。
- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)：第三方来源和许可证说明。

## 许可证

本仓库自己的修改按 [Apache License 2.0](LICENSE) 发布。第三方源码、模型和资源继续受其原始许可证约束，详情见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
