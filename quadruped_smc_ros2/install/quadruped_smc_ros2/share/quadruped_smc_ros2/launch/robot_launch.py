import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('quadruped_smc_ros2')

    default_world = os.path.join(package_dir, 'resource', 'robot_8dof.wbt')
    urdf_path     = os.path.join(package_dir, 'urdf', 'robot_description.urdf')

    robot_description = ''
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
    else:
        robot_description = '<robot name="quadruped"/>'

    # ── Arguments ─────────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to Webots world file (.wbt)'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Webots'
    )

    # ── Webots Driver Node ────────────────────────────────────
    webots_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=[
            '--webots-world', LaunchConfiguration('world'),
        ]
    )

    # ── Robot State Publisher ─────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ── Keyboard Node ─────────────────────────────────────────
    gait_keyboard = Node(
        package='quadruped_smc_ros2',
        executable='gait_keyboard_node',
        output='screen',
        prefix='gnome-terminal --',
    )

    # ── Error Monitor Node ────────────────────────────────────
    error_monitor = Node(
        package='quadruped_smc_ros2',
        executable='error_monitor_node',
        output='screen',
        prefix='gnome-terminal --',
    )

    # ── Shutdown when Webots exits ────────────────────────────
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=webots_driver,
            on_exit=[
                LogInfo(msg='Webots closed, shutting down...'),
                EmitEvent(event=Shutdown())
            ]
        )
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,

        LogInfo(msg='[Quadruped SMC] Starting Webots + ROS2 controller...'),
        LogInfo(msg='[Quadruped SMC] Keyboard : T=Trot B=Bound P=Pace W=Walk Q=Quit'),
        LogInfo(msg='[Quadruped SMC] Monitor  : R=Reset stats  Q=Quit'),
        # LogInfo(msg='[Quadruped SMC] Plot 1   : FL_hip FL_knee FR_hip FR_knee'),
        # LogInfo(msg='[Quadruped SMC] Plot 2   : BR_hip BR_knee BL_hip BL_knee'),

        webots_driver,
        robot_state_publisher,
        gait_keyboard,
        error_monitor,
        shutdown_handler,
    ])