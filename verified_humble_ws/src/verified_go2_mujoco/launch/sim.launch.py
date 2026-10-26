from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('go2_description'))
    adapter = Path(get_package_share_directory('verified_go2_mujoco'))
    robot = xacro.process_file(
        str(description / 'xacro/robot.xacro'),
        mappings={'HARDWARE_PLUGIN': 'mujoco_ros2_control/MujocoSystem'},
    ).toxml()
    config = str(description / 'config/robot_control.yaml')

    simulator = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            config,
            {
                'robot_description': robot,
                'mujoco_model_path': LaunchConfiguration('model'),
                'initial_keyframe': 'home',
                'base_body_name': 'base_link',
                'base_frame_id': 'base',
                'odom_frame_id': 'odom',
                'headless': LaunchConfiguration('headless'),
            },
        ],
        additional_env={
            'GO2_RECOMPILE_CPPAD': LaunchConfiguration('recompile_cppad'),
        },
    )
    publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot, 'use_sim_time': True}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_ocs2', output='screen',
        arguments=['-d', str(adapter / 'config/go2.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    joy = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_device'),
            'deadzone': ParameterValue(LaunchConfiguration('joy_deadzone'), value_type=float),
            'autorepeat_rate': 20.0,
            'coalesce_interval_ms': 1,
        }],
        condition=IfCondition(LaunchConfiguration('joystick')),
    )
    joystick_input = Node(
        package='joystick_input', executable='joystick_input',
        name='joystick_input', output='screen',
        parameters=[{
            'deadzone': ParameterValue(LaunchConfiguration('joy_deadzone'), value_type=float)
        }],
        condition=IfCondition(LaunchConfiguration('joystick')),
    )

    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '600'],
        )

    joints = spawner('joint_state_broadcaster')
    imu = spawner('imu_sensor_broadcaster')
    locomotion = spawner('ocs2_quadruped_controller')

    def next_on_success(next_action):
        def callback(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='controller activation failed'))]
            return [next_action] if next_action is not None else []
        return callback

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=str(adapter / 'mjcf/scene.xml'),
            description='MuJoCo MJCF scene owned by this project.'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joystick', default_value='false'),
        DeclareLaunchArgument(
            'recompile_cppad', default_value='false',
            description='Regenerate OCS2 CppAD libraries before starting MPC.'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.08'),
        simulator, publisher, rviz, joy, joystick_input, joints,
        RegisterEventHandler(OnProcessExit(
            target_action=joints, on_exit=next_on_success(imu))),
        RegisterEventHandler(OnProcessExit(
            target_action=imu, on_exit=next_on_success(locomotion))),
        RegisterEventHandler(OnProcessExit(
            target_action=locomotion, on_exit=next_on_success(None))),
        RegisterEventHandler(OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='MuJoCo exited'))])),
    ])
