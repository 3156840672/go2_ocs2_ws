# 雷神 G30S 手柄

加载本工作空间 env.sh 后运行 `ros2 launch joystick_input joystick.launch.py`。

LB+A：趴下/站立/退出trot；LB+Y：站立后进入trot；LB+B：被动模式。两次LB+A之间松开并等待至少2秒。左摇杆平移，右摇杆横向转向。不要同时运行键盘节点。

映射以本包 `src/JoystickInput.cpp` 为准：A/B/X/Y=0/1/3/4，LB=6；左摇杆轴0/1，右摇杆轴2/3，LT按钮8/轴5。完整操作和验证范围见根目录 USAGE.md。
