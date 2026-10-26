# Go2：ROS 2 Humble + OCS2 + WBC + MuJoCo

当前主路线已经切换为本机 **ROS 2 Humble**，不使用 Docker。控制器来自已验证项目 `/home/yue/Downloads/quadruped_ros2_control-humble`，核心控制链全部是 C++：

```text
键盘/手柄 -> /control_input -> OCS2 SQP-NMPC -> Weighted WBC
             -> q_des, dq_des, tau_ff, kp, kd -> MuJoCo -> 状态估计
```

项目包含 `ocs2_quadruped_controller`、Go2 的 OCS2 参数和自有 MuJoCo 模型。默认后端使用 `verified_go2_mujoco/mjcf/scene.xml`，通过进程内 `mujoco_ros2_control` 直接接入 ros2_control；qpOASES 与其他 OCS2 底层库由本机 Humble underlay 提供。

## 已验收

- 9 个当前路线软件包在 Humble 下编译通过。
- OCS2 控制器、关节状态广播器、IMU 广播器均为 `active`。
- SQP-MPC 收到初始策略，Weighted WBC 实时输出五路电机命令。
- 固定站立和零速 trot 均保持约 `0.348 m` 高度。
- 已按“passive 自然趴下 -> command 2 起立”的真实流程验收，起立后高度约 `0.342 m`。
- `ly=0.30` 的 trot 连续运行 10 秒，前进约 `0.96 m`、高度保持 `0.349 m`，随后正常收步并切回 stance。
- command `8` 的 dynamic_walk 和 command `9` 的 static_walk 均完成 10 秒行走测试，并可正常切回 stance。
- RViz 使用完整 Go2 网格，并显示优化质心/足端轨迹、未来落足点、足端力、压力中心和支撑多边形。
- RViz 的 `odom -> base` 只由 OCS2 状态估计发布，MuJoCo 真值通过 `/go2/base_odom` 单独提供，不再产生冲突 TF。
- 手柄输入已接入统一启动文件，并验证 `Joy -> /control_input -> OCS2` 映射。

## 快速启动

终端 A：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true
```

终端 B：

```bash
cd /home/yue/Desktop/go2_ocs2_ws/verified_humble_ws
source env.sh
# 不用手柄时才运行键盘节点
ros2 run keyboard_input keyboard_input
```

启动后机器人会在 passive 状态自然趴下。等待控制器加载完成，再按 `2` 从趴姿进入 OCS2 stance；按 `3` 进入 trot，按 `W` 逐步增加前进速度，空格清零速度，最后按 `2` 回到 stance。

- [PROJECT_GUIDE.md](PROJECT_GUIDE.md)：从首次编译到 OCS2、状态估计和 WBC 原理的完整逐步教程。
- [USAGE.md](USAGE.md)：日常启动命令、按键表和快速故障排查。
