# Go2 Humble + OCS2 使用说明

本文用于日常操作速查。第一次使用或需要理解完整控制原理时，请阅读 [PROJECT_GUIDE.md](PROJECT_GUIDE.md)。

## 1. 工程组成

主工作空间是 `/home/yue/Desktop/go2_ocs2_ws/verified_humble_ws`。

| 目录 | 作用 |
| --- | --- |
| `src/quadruped_ros2_control/controllers/ocs2_quadruped_controller` | C++ OCS2、SQP-MPC、状态估计和 Weighted WBC |
| `src/quadruped_ros2_control/descriptions/unitree/go2_description` | Go2 URDF、网格、控制器与 OCS2 参数 |
| `src/verified_go2_mujoco` | 本项目 Go2 MJCF、OCS2 RViz 配置和统一启动文件 |
| `src/mujoco_ros2_control` | 默认进程内 MuJoCo 到 ros2_control 的 C++ 硬件桥 |

`env.sh`依次加载 `/opt/ros/humble`、`/home/yue/ros2_ws/install` 中已编译的 Humble OCS2 底层库，以及当前工作空间。默认模型是当前工程的 `src/verified_go2_mujoco/mjcf/scene.xml`，仿真和 ros2_control 在同一 C++ 进程中运行，不依赖 `/home/yue/unitree_mujoco` 或 Unitree DDS。整个链路都是本机 Humble，不经过 Docker。

## 2. 编译

打开一个新的终端：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
MAKEFLAGS=-j2 colcon build --base-paths src \
  --packages-up-to joystick_input ocs2_quadruped_controller verified_go2_mujoco \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source env.sh
```

首次完整编译需要几分钟。qpOASES 复用 underlay 中已经验证的版本，不会在本项目内重新下载。CppAD 模型库位于 `~/ocs2_cpp_ad/go2`；本机已有这些库，启动时会直接加载。

## 3. 启动

终端 A：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py
```

该命令依次启动项目内 MuJoCo 模型、进程内 ros2_control、robot_state_publisher、OCS2 控制器和 RViz。其他模式：

```bash
# 不打开 RViz；MuJoCo 窗口仍会打开
ros2 launch verified_go2_mujoco sim.launch.py rviz:=false

# 无窗口运行
ros2 launch verified_go2_mujoco sim.launch.py headless:=true rviz:=false

# 临时加载另一个自有 MJCF 场景
ros2 launch verified_go2_mujoco sim.launch.py model:=/绝对路径/scene.xml

```

MuJoCo 窗口按 `V` 切换外观网格，按 `C` 切换碰撞体。默认 MJCF 源文件为 `verified_humble_ws/src/verified_go2_mujoco/mjcf/scene.xml`，修改后需要重新编译 `verified_go2_mujoco` 或通过 `model:=` 直接加载源码绝对路径。RViz Fixed Frame 为 `odom`。

RViz 默认显示：

- `Optimized State Trajectory`：预测质心轨迹、四足轨迹和未来落足点。
- `Current State`：足端位置/力、压力中心和支撑多边形。
- `Target Trajectories`：目标基座轨迹和四足目标轨迹。
- `RobotModel` 与 `TF`：完整 Go2 网格和 `odom -> base -> trunk -> legs` 变换树。

## 4. 键盘控制 OCS2

终端 B：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 run keyboard_input keyboard_input
```

操作顺序：

1. 启动后机器人处于 passive，会在重力作用下自然趴到地面；这是预期初始状态。
2. 控制器加载完成后按 `2`，从趴姿进入 OCS2 stance；首次进入会等待初始 MPC 策略。
3. 机器人稳定站立后按 `3`，切换到 trot。
4. 按 `W` 增加前进输入；第一次测试只按 1 至 2 次。
5. `A/D` 控制侧向，`J/L` 控制偏航。
6. 按空格清零全部速度，等待收步。
7. 按 `2` 回到 stance，再用 `Ctrl+C` 结束两个终端。

| 按键/command | OCS2 行为 |
| --- | --- |
| `1` | passive，会失去主动支撑 |
| `2` | stance |
| `3` | trot |
| `4` | standing_trot |
| `5` | flying_trot |
| `6` | pace |
| `7` | standing_pace |
| `8` | dynamic_walk |
| `9` | static_walk |
| `0` / command `10` | amble |
| 空格 | 速度清零，步态不变 |

键盘输入是增量保持式的，松开 `W` 不会自动归零。默认满前进输入对应约 `0.5 m/s`。

`gait.info` 还包含 command `11` 至 `13` 的 `lindyhop`、`skipping` 和 `pawup` 模板，但当前键盘节点没有为它们分配按键，也尚未在自有 MuJoCo 模型上验收。

## 5. 雷神 G30S 手柄

键盘和手柄节点不要同时运行。先把手柄接到 `/dev/input/js0`，然后在终端 A 直接启动仿真和手柄：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true
```

| 手柄 | command | OCS2 行为 |
| --- | --- | --- |
| `LB+B` | 1 | passive |
| `LB+A` | 2 | stance |
| `LB+X` | 3 | trot |
| `LB+Y` | 4 | standing_trot |
| `LT+B` | 5 | flying_trot |
| `LT+A` | 6 | pace |
| `LT+X` | 7 | standing_pace |
| `LT+Y` | 8 | dynamic_walk |
| `START` | 9 | static_walk |

左摇杆控制平移，右摇杆横向控制偏航。先按 `LB+A` 站立，再按 `LB+X` 进入 trot；结束时摇杆回中，再按 `LB+A` 回 stance。

当前 G30S TE 实测映射：A/B/X/Y=`0/1/3/4`，LB=`6`，左摇杆轴=`0/1`，右摇杆轴=`2/3`，LT 按钮=`8`、轴=`5`。启动后先让两个摇杆回中，速度输出才会解除锁定。默认死区为 `0.08`；可用 `joy_device:=/dev/input/js1 joy_deadzone:=0.12` 修改。换手柄后先检查 `ros2 topic echo /joy --once`。

## 6. 控制链原理

```text
关节位置/速度 + IMU + 四足法向力
                  |
线性卡尔曼状态估计 -> OCS2 SystemObservation
                  |
SQP-NMPC（目标 100 Hz）预测未来 1 秒状态、接触模式和地面反力
                  |
控制更新循环通过 MRT 读取最新策略
                  |
Weighted WBC 求解全身 QP
  浮动基动力学、摩擦锥、接触、摆腿、力矩限制
                  |
12 关节 tau_ff、q_des、dq_des、kp、kd
                  |
MuJoCo: tau = tau_ff + kp(q_des-q) + kd(dq_des-dq)
```

NMPC 是滚动优化控制框架，SQP 是每次更新时求解非线性最优控制问题的方法，两者不是二选一。WBC 将 NMPC 的状态、接触与地面反力目标变成满足整机动力学约束的关节命令。

默认后端在 MuJoCo 每个仿真周期直接读取关节、IMU 和四足接触力，经 ros2_control 状态接口交给估计器；WBC 输出的五路混合命令在同一进程中按 `tau = tau_ff + kp(q_des-q) + kd(dq_des-dq)` 施加到 12 个关节。四足接触力按接口名字匹配 `FL/FR/RL/RR` 几何体，不依赖 Unitree `LowState` 的数组顺序。

RViz 的 `odom -> base` 只由 OCS2 状态估计可视化器发布。MuJoCo 真值保留在 `/go2/base_odom`，但不再重复发布同名 TF；否则 RViz 会在真值和估计值之间切换，表现为完整机器人模型闪烁。

## 7. 检查命令

```bash
# 三个控制器应均为 active
ros2 control list_controllers

# 控制器卡尔曼估计
ros2 topic echo /odom --once

# OCS2 预测轨迹和未来落足点
ros2 topic echo /legged_robot/optimizedStateTrajectory --once

# 足端力、压力中心和支撑多边形
ros2 topic echo /legged_robot/currentState --once

# 确认 odom 到完整机器人模型的 TF
ros2 run tf2_ros tf2_echo odom base

# /tf 应只有 ocs2_quadruped_controller 和 robot_state_publisher 两个发布者
ros2 topic info /tf --verbose --no-daemon
```

本轮自有 MuJoCo 模型验收采用真实操作顺序：先让 passive 状态自然趴下（机身高度约 `0.069 m`），再发送 command `2` 起立。站立高度约 `0.342 m`；`ly=0.30` 的 trot 连续运行 10 秒后高度为 `0.349 m`、前进约 `0.96 m`。command `8` 的 dynamic_walk 和 command `9` 的 static_walk 也分别完成 10 秒行走并正常切回 stance，未出现 WBC/QP 或安全检查错误。`Future footholds` 实测包含 6 个三维落足点。

## 8. 常见问题

- **包路径错误：**新开终端，进入 `verified_humble_ws` 后只执行 `source env.sh`；用 `ros2 pkg prefix ocs2_quadruped_controller` 检查。
- **首次加载慢：**CppAD 首次生成动态库会耗时；控制器加载超时已放宽到 600 秒。
- **线程优先级警告：**普通用户没有实时调度权限时会警告，但本轮控制器仍正常求解和行走。
- **松开 W 仍继续走：**这是增量保持输入，按空格清零，再按 `2` 回 stance。
- **启动后机器人趴下：**这是预期行为。启动状态是 passive，仿真仍受重力作用；等控制器加载完成后按 `2`，机器人会从趴姿进入 stance。
- **切换 trot 后翻倒：**先确认自有 MJCF 的足端碰撞几何及 ros2_control 接口都按 OCS2 顺序 `FL/FR/RL/RR` 映射，再检查摩擦、质量、惯量和初始关节角是否与 OCS2 的 URDF/参数一致。
- **找不到手柄：**运行 `ls /dev/input/js*`；如果实际是 `js1`，启动时增加 `joy_device:=/dev/input/js1`。当前机器验收时没有连接真实手柄，但已用合成 `sensor_msgs/Joy` 验证完整映射节点。
- **RViz 只有模型：**必须使用 `verified_go2_mujoco/config/go2.rviz`，Fixed Frame 为 `odom`，并检查 `/legged_robot/optimizedStateTrajectory` 和 `/legged_robot/currentState`。
- **RViz 模型闪烁：**先结束所有旧的 launch，再重新 `source env.sh` 启动。运行 `ros2 topic info /tf --verbose --no-daemon`，发布者应只有 `ocs2_quadruped_controller` 和 `robot_state_publisher`；出现 `mujoco_ros2_control_node` 或重复节点说明旧进程仍在发布冲突 TF。
- **没有未来落足点：**stance 下 `Future footholds` 为空是正常的；切换到 trot 后才会出现。
- **窗口打不开：**图形模式需要桌面和 `DISPLAY`；无桌面环境使用 `ros2 launch verified_go2_mujoco sim.launch.py headless:=true rviz:=false`。
