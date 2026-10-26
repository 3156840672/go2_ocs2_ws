# Native Humble Go2 workspace

This workspace runs the C++ `ocs2_quadruped_controller` and Weighted WBC against the project-owned MuJoCo model in `src/verified_go2_mujoco/mjcf`. The default launch uses the in-process `mujoco_ros2_control` backend and can start joystick control together with the simulator.

```bash
source env.sh
ros2 launch verified_go2_mujoco sim.launch.py joystick:=true
```

OCS2 libraries come from `/home/yue/ros2_ws/install`; controller source, Go2 configuration, MJCF model, and ROS 2 simulation bridge are built here. Full instructions are in [../USAGE.md](../USAGE.md).

The complete Chinese walkthrough and architecture explanation is in [../PROJECT_GUIDE.md](../PROJECT_GUIDE.md).
