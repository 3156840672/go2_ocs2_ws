# Go2 模型

来自已验证的下载项目。`xacro/ros2_control.xacro` 已适配本工作空间的 `mujoco_ros2_control/MujocoSystem`，保留关节和惯性等原模型参数。控制配置只保留 OCS2/WBC 链路。

运行入口：`ros2 launch verified_go2_mujoco sim.launch.py headless:=false`。手柄入口：`ros2 launch joystick_input joystick.launch.py`。完整步骤见工作空间根目录 USAGE.md。
