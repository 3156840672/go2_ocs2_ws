# 输入节点

`keyboard_input` 或 `joystick_input` 向 `/control_input` 发布控制命令。两种方式只启动一个。G30S手柄通过 `joystick_input/joystick.launch.py` 同时启动设备读取和消息转换；具体操作见根目录 USAGE.md。
