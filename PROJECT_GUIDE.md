# Go2 OCS2 项目完整教程

本文从实际使用和控制原理两个角度介绍当前项目。默认路线为：本机 ROS 2 Humble、自有 Go2 MuJoCo 模型、C++ OCS2 控制器、线性卡尔曼状态估计和 Weighted WBC，不使用 Docker，也不依赖 Unitree DDS。

## 1. 先认识最终系统

项目运行时的数据流如下：

```text
键盘 / 手柄
    |
    v
/control_input
    |
    +--> 步态管理器：stance、trot 等接触时序
    |
    +--> 目标管理器：期望速度、方向、偏航和高度
              |
              v
关节状态 + IMU + 四足法向力 --> 线性卡尔曼状态估计
              |                       |
              |                       v
              +--------------> OCS2 SystemObservation
                                      |
                                      v
                           SQP-NMPC，100 Hz
                         预测未来 1 秒运动和足端力
                                      |
                                      v
                              Weighted WBC
                    将优化结果转换为 12 个关节命令
                                      |
                                      v
                  tau_ff、q_des、dq_des、kp、kd
                                      |
                                      v
                         ros2_control 硬件接口
                                      |
                                      v
                                  MuJoCo
```

这里没有 Guide 控制器。默认控制核心是 `ocs2_quadruped_controller`。

## 2. 目录和关键文件

主工作空间：

```text
/home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
```

| 路径 | 作用 |
| --- | --- |
| `src/verified_go2_mujoco/mjcf/scene.xml` | 默认 MuJoCo 场景入口 |
| `src/verified_go2_mujoco/mjcf/go2.xml` | 自有 Go2 刚体、碰撞、惯量和执行器模型 |
| `src/verified_go2_mujoco/launch/sim.launch.py` | 仿真、控制器、RViz 和手柄统一启动文件 |
| `src/mujoco_ros2_control` | MuJoCo 与 ros2_control 之间的 C++ 桥 |
| `src/quadruped_ros2_control/controllers/ocs2_quadruped_controller` | 状态估计、OCS2、FSM 和 WBC |
| `src/quadruped_ros2_control/descriptions/unitree/go2_description/urdf/robot.urdf` | OCS2 和 Pinocchio 使用的机器人模型 |
| `src/quadruped_ros2_control/descriptions/unitree/go2_description/config/robot_control.yaml` | 控制器、传感器和接口配置 |
| `src/quadruped_ros2_control/descriptions/unitree/go2_description/config/ocs2/task.info` | NMPC、代价、约束和求解器参数 |
| `src/quadruped_ros2_control/descriptions/unitree/go2_description/config/ocs2/reference.info` | 默认高度、关节姿态和速度上限 |
| `src/quadruped_ros2_control/descriptions/unitree/go2_description/config/ocs2/gait.info` | stance、trot 等步态模板 |
| `env.sh` | 加载 Humble、OCS2 underlay 和当前工作空间 |

## 3. 第一次编译

### 3.1 打开干净终端

打开一个新的终端，执行：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
```

确认环境：

```bash
echo $ROS_DISTRO
echo $ROS_DOMAIN_ID
ros2 pkg prefix ocs2_quadruped_controller
```

预期：

```text
humble
42
/home/yue/Desktop/go2_ocs2_ws/verified_humble_ws/install/ocs2_quadruped_controller
```

### 3.2 编译全部十个包

```bash
MAKEFLAGS=-j2 colcon build --base-paths src \
  --executor sequential \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

编译成功后重新加载环境：

```bash
source env.sh
```

以后只修改单个包时可以缩短编译时间。例如：

```bash
colcon build --packages-select mujoco_ros2_control \
  ocs2_quadruped_controller --symlink-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source env.sh
```

## 4. 第一次启动：使用键盘

### 4.1 终端 A：启动仿真、OCS2 和 RViz

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py
```

启动文件按顺序完成以下工作：

1. 加载 `scene.xml` 和 Go2 MJCF。
2. 创建进程内 `mujoco_ros2_control` 硬件系统。
3. 启动 `robot_state_publisher`。
4. 激活 `joint_state_broadcaster`。
5. 激活 `imu_sensor_broadcaster`。
6. 加载 OCS2 模型、CppAD 动力学库和 SQP 求解器。
7. 激活 `ocs2_quadruped_controller`。
8. 打开 RViz，加载本项目配置。

等待终端出现类似信息：

```text
MRT initialized. MPC thread started.
Using Kalman Filter Estimator
Configured and activated ocs2_quadruped_controller
```

启动后机器人处于 passive。MuJoCo 已经受重力作用，所以机器人自然趴下是正常现象，不是控制失败。

### 4.2 终端 B：启动键盘节点

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 run keyboard_input keyboard_input
```

### 4.3 严格按照以下顺序操作

1. 等机器人自然趴下。
2. 等终端 A 显示 OCS2 控制器已经激活。
3. 在终端 B 按 `2`，进入 stance。
4. 等机器人完全站稳。
5. 按 `3`，切换到 trot。
6. 按一到两次 `W`，小幅增加前进速度。
7. 观察 MuJoCo 中的真实运动和 RViz 中的估计状态、预测轨迹及落足点。
8. 按空格，将速度输入清零。
9. 等机器人收步后按 `2`，回到 stance。
10. 先停止键盘节点，再在终端 A 按 `Ctrl+C` 停止仿真。

不要从高速运动直接切换到 passive。按 `1` 会撤掉主动支撑，机器人会再次趴下。

## 5. 键盘含义

| 按键 | 作用 |
| --- | --- |
| `1` | passive |
| `2` | stance |
| `3` | trot |
| `4` | standing_trot |
| `5` | flying_trot |
| `6` | pace |
| `7` | standing_pace |
| `8` | dynamic_walk |
| `9` | static_walk |
| `0` | amble，内部 command 为 `10` |
| `W/S` | 增加/减小前向速度输入 |
| `A/D` | 调整侧向速度输入 |
| `J/L` | 调整偏航速度输入 |
| `I/K` | 调整竖直方向输入 |
| 空格 | 将四个连续输入清零 |

键盘输入是增量保持式的。松开 `W` 不会自动停止，必须按空格清零。

默认参数：

- 满量程平移速度：`0.5 m/s`
- 满量程偏航速度：`1.57 rad/s`
- 目标质心高度：`0.35 m`

## 6. 使用雷神 G30S 手柄

### 6.1 检查设备

```bash
ls /dev/input/js*
```

假设设备为 `/dev/input/js0`，直接启动：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true
```

手柄模式下不要再启动键盘节点，以免两个节点同时写 `/control_input`。

### 6.2 手柄映射

| 手柄操作 | 控制命令 |
| --- | --- |
| `LB+B` | passive |
| `LB+A` | stance |
| `LB+X` | trot |
| `LB+Y` | standing_trot |
| `LT+B` | flying_trot |
| `LT+A` | pace |
| `LT+X` | standing_pace |
| `LT+Y` | dynamic_walk |
| `START` | static_walk |
| 左摇杆 | 前后和侧向速度 |
| 右摇杆横向 | 偏航速度 |

操作流程：

1. 启动后先让左右摇杆回中，解除速度输出安全锁定。
2. 等机器人自然趴下且控制器加载完成。
3. 按 `LB+A` 起立。
4. 站稳后按 `LB+X` 进入 trot。
5. 从很小的左摇杆输入开始。
6. 停止时先让摇杆回中。
7. 按 `LB+A` 回到 stance。

修改设备和死区：

```bash
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true \
  joy_device:=/dev/input/js1 joy_deadzone:=0.12
```

## 7. 一次控制周期里发生了什么

各层采用不同频率，因为它们的计算量和职责不同：

| 层级 | 当前配置 | 职责 |
| --- | --- | --- |
| MuJoCo 物理 | `1000 Hz`，步长 `0.001 s` | 积分刚体动力学和接触 |
| controller manager | `1000 Hz` | 读写 ros2_control 硬件接口 |
| OCS2 控制器 | `500 Hz` | 状态更新、MRT 与 WBC |
| SQP-NMPC | 目标 `100 Hz` | 更新未来 1 秒最优策略 |

高频 WBC 不会每次重新求解完整 NMPC。它通过 MRT 使用 MPC 线程最近一次完成的策略，因此昂贵的预测优化和快速的关节控制可以解耦。

### 7.1 MuJoCo 产生模拟传感器数据

仿真步长为 `0.001 s`。MuJoCo 计算：

- 12 个关节的位置、速度和力矩
- IMU 姿态、角速度和线加速度
- `FL、FR、RL、RR` 四个足端的法向接触力
- 基座真实位置和速度

前三类数据通过 ros2_control 状态接口交给控制器。基座真值只发布到 `/go2/base_odom`，用于调试，不进入默认控制闭环。

### 7.2 卡尔曼滤波器估计基座状态

默认配置是：

```yaml
estimator_type: linear_kalman
```

估计器使用关节运动学、IMU 和足端接触状态估计：

- 基座位置与速度
- 基座姿态与角速度
- 足端在世界坐标系中的位置
- 当前接触模式

估计结果发布到 `/odom`，并转换为 OCS2 的 `SystemObservation`。

因此：

| 数据 | 含义 | 是否进入默认控制 |
| --- | --- | --- |
| `/odom` | 卡尔曼估计 | 是 |
| `/go2/base_odom` | MuJoCo 真值 | 否，仅诊断 |
| `odom -> base` TF | OCS2 估计姿态 | 用于 RViz |

### 7.3 输入变成运动目标

`TargetManager` 将摇杆或键盘值转换为目标：

```text
前向目标速度 = ly * 0.5 m/s
侧向目标速度 = -lx * 0.5 m/s
偏航目标速度 = -rx * 1.57 rad/s
目标高度     = 0.35 m
```

它不是直接给电机发送速度，而是生成预测时域末端的基座目标。

### 7.4 步态管理器给出接触计划

`GaitManager` 根据 command 选择 `gait.info` 中的模板：

| command | 步态 | 当前输入入口 | 当前状态 |
| --- | --- | --- | --- |
| `2` | stance | 键盘 `2` / `LB+A` | 已验收 |
| `3` | trot | 键盘 `3` / `LB+X` | 已验收 |
| `4` | standing_trot | 键盘 `4` / `LB+Y` | 模板已接入 |
| `5` | flying_trot | 键盘 `5` / `LT+B` | 模板已接入 |
| `6` | pace | 键盘 `6` / `LT+A` | 模板已接入 |
| `7` | standing_pace | 键盘 `7` / `LT+X` | 模板已接入 |
| `8` | dynamic_walk | 键盘 `8` / `LT+Y` | 已验收 |
| `9` | static_walk | 键盘 `9` / `START` | 已验收 |
| `10` | amble | 键盘 `0` | 模板已接入 |
| `11` | lindyhop | 当前无按键 | 仅模板 |
| `12` | skipping | 当前无按键 | 仅模板 |
| `13` | pawup | 当前无按键 | 仅模板 |

`dynamic_walk` 在一个周期中混合两足和三足支撑，速度较快；`static_walk` 始终保持三足支撑，周期更长，通常更适合低速稳定行走。`flying_trot` 和 `pace` 等更激进步态虽然已经进入 OCS2 步态表，但不应等同于已在当前自有模型上完成验收。

步态模板只规定“哪条腿在什么时间接触”，不会直接计算电机力矩。

### 7.5 OCS2 执行 NMPC

NMPC 每次使用当前估计状态，在未来 `1.0 s` 内优化：

- 基座位置、速度和姿态
- 12 个关节状态
- 四足地面反力
- 已知步态接触时序下的运动轨迹

当前求解器参数：

```text
算法：Multiple Shooting SQP
期望 MPC 频率：100 Hz
离散步长：0.015 s
每次 MPC 更新 SQP 迭代：1
预测时域：1.0 s
```

NMPC 是“滚动预测控制框架”，SQP 是“求解这个非线性优化问题的数值方法”，两者不是替代关系。

### 7.6 Weighted WBC 生成关节命令

NMPC 给出期望整机状态和足端力，但电机需要关节级命令。Weighted WBC 通过 QP 同时处理：

- 浮动基动力学一致性
- 支撑腿接触约束
- 摩擦锥约束
- 摆动腿轨迹
- 期望基座运动
- 关节和力矩限制

输出给每个关节的五项命令是：

```text
tau_ff：前馈力矩
q_des：期望角度
dq_des：期望角速度
kp：位置增益
kd：速度增益
```

MuJoCo 最终施加：

```text
tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)
```

## 8. RViz 中每项内容代表什么

RViz 固定坐标系为 `odom`。

| 显示项 | 含义 |
| --- | --- |
| RobotModel | 根据估计基座 TF 和实际关节状态绘制的完整 Go2 |
| Optimized State Trajectory | NMPC 对未来基座和足端状态的预测 |
| Current State | 当前足端位置、接触力、压力中心和支撑多边形 |
| Target Trajectories | 控制输入生成的目标轨迹 |
| Future footholds | 动态步态中预测的未来落足点 |

stance 下未来落足点为空或数量很少是正常的；切换到 trot 后才会出现明显落足序列。

`odom -> base` 只能由 OCS2 可视化器发布。MuJoCo 真值保留在 `/go2/base_odom`，不能再发布同名 TF，否则 RViz 会在真值和估计值之间跳变。

## 9. 每次运行后的检查

新开终端并加载环境：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
```

检查控制器：

```bash
ros2 control list_controllers
```

应看到三个 `active`：

```text
joint_state_broadcaster
imu_sensor_broadcaster
ocs2_quadruped_controller
```

检查估计和真值：

```bash
ros2 topic echo /odom --once
ros2 topic echo /go2/base_odom --once
```

检查足端接触：

```bash
ros2 topic echo /go2/foot_contacts --once
```

检查 OCS2 可视化输出：

```bash
ros2 topic echo /legged_robot/optimizedStateTrajectory --once
ros2 topic echo /legged_robot/currentState --once
```

检查 TF 冲突：

```bash
ros2 topic info /tf --verbose --no-daemon
```

发布者数量应为 2：

```text
ocs2_quadruped_controller
robot_state_publisher
```

检查模型一致性：

```bash
python3 src/verified_go2_mujoco/tools/validate_model.py \
  src/verified_go2_mujoco/mjcf/scene.xml
```

输出中的 `passed` 应为 `true`。

## 10. 常用启动变体

只关闭 RViz：

```bash
ros2 launch verified_go2_mujoco sim.launch.py rviz:=false
```

完全无窗口：

```bash
ros2 launch verified_go2_mujoco sim.launch.py headless:=true rviz:=false
```

临时加载另一个 MJCF：

```bash
ros2 launch verified_go2_mujoco sim.launch.py model:=/绝对路径/scene.xml
```

## 11. 故障排查顺序

### 11.1 RViz 模型闪烁

1. 停止所有旧 launch。
2. 新开终端并执行 `source env.sh`。
3. 重新启动项目。
4. 检查 `/tf` 发布者数量。

如果出现 `mujoco_ros2_control_node` 发布 `/tf`，说明运行的是旧二进制或旧进程。

### 11.2 机器人无法起立

依次检查：

```bash
ros2 control list_controllers
ros2 topic echo /imu_sensor_broadcaster/imu --once
ros2 topic echo /go2/foot_contacts --once
ros2 topic echo /control_input --once
```

确认按 `2` 后日志出现：

```text
Switched from passive to OCS2 State
Switch to gait: stance
```

### 11.3 起立正常，但 trot 不稳定

1. 先在 stance 等待 3 至 5 秒。
2. 切换 trot 时保持速度为零。
3. 零速 trot 稳定后只增加很小前进输入。
4. 检查足端顺序必须为 `FL、FR、RL、RR`。
5. 检查 `/odom` 与 `/go2/base_odom` 的高度和姿态是否接近。
6. 检查日志中是否出现 `Safety check failed`、QP 错误或 `NaN`。

### 11.4 找不到手柄

```bash
ls /dev/input/js*
ros2 topic echo /joy --once
```

如果手柄是 `js1`，使用：

```bash
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true \
  joy_device:=/dev/input/js1
```

### 11.5 停止后仍有旧节点

先查看：

```bash
ros2 node list
```

回到原启动终端按 `Ctrl+C`，等待所有子进程退出后再重新启动。不要同时运行两份 `sim.launch.py`。

## 12. 当前验收基线

当前版本已按以下完整流程验证：

1. passive 状态自然趴下，机身高度约 `0.069 m`。
2. command `2` 从趴姿起立，站立高度约 `0.342 m`。
3. `ly=0.30` 的 trot 连续运行 10 秒。
4. 前进约 `0.96 m`，运动高度约 `0.348 m`。
5. 切回 stance 后高度约 `0.347 m`。
6. 未出现 WBC/QP 或安全检查错误。
7. 站立高度标准差约 `0.27 mm`。
8. trot 高度标准差约 `0.80 mm`，横滚和俯仰峰值小于 `0.47°`。
9. OCS2 URDF 与 MuJoCo 刚体质量矩阵误差约 `4.81e-8`。
10. `/tf` 只保留 OCS2 和 robot_state_publisher 两个发布者。
11. command `8` dynamic_walk 连续运行 10 秒并正常回到 stance。
12. command `9` static_walk 连续运行 10 秒并正常回到 stance。

## 13. 推荐学习顺序

为了真正理解代码，建议按以下顺序阅读：

1. `sim.launch.py`：了解系统启动了哪些节点。
2. `robot_control.yaml`：了解控制器需要哪些硬件接口。
3. `mujoco_system.cpp`：了解模拟传感器和电机命令怎样交换。
4. `Ocs2QuadrupedController.cpp`：了解 ros2_control 生命周期和状态机入口。
5. `StateEstimateBase.cpp` 与 `LinearKalmanFilter.cpp`：了解观测状态怎样产生。
6. `TargetManager.cpp`：了解摇杆怎样变成目标运动。
7. `GaitManager.cpp` 与 `gait.info`：了解接触模式怎样切换。
8. `StateOCS2.cpp`：了解 MPC 策略怎样进入 WBC。
9. `WeightedWbc.cpp`：了解全身 QP 怎样生成关节力矩。
10. `task.info`：最后再调整权重、约束和求解器参数。

先保证默认参数可以稳定复现，再一次只修改一类参数，并记录修改前后的站立高度、姿态波动、足端力和求解器日志。
