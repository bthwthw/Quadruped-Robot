import os
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo,
                             RegisterEventHandler, EmitEvent, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node as RosNode
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────────
#  Sequence
# ─────────────────────────────────────────────────────────────
GAIT_SEQUENCE = [
    ("trot",  5.0),
    ("bound", 5.0),
    ("pace",  5.0),
    ("walk",  5.0),
]
STARTUP_DELAY = 5.0   # Đợi Webots + driver load xong
# ─────────────────────────────────────────────────────────────


def run_gait_sequence():
    time.sleep(STARTUP_DELAY)

    rclpy.init()
    node = Node("gait_sequence_node")
    pub  = node.create_publisher(String, "/gait_command", 10)

    total = sum(d for _, d in GAIT_SEQUENCE)
    print("\n╔══════════════════════════════════════════╗")
    print("║       Quadruped  —  Sequence Mode        ║")
    print("╠══════════════════════════════════════════╣")
    for name, dur in GAIT_SEQUENCE:
        print(f"║  {name.upper():<10}  {dur:>4.1f}s                       ║")
    print(f"╠══════════════════════════════════════════╣")
    print(f"║  Tổng: {total:.1f}s                             ║")
    print("╚══════════════════════════════════════════╝\n")

    for gait, duration in GAIT_SEQUENCE:
        node.get_logger().info(f"▶ Gait → {gait.upper()}  ({duration:.1f}s)")

        steps = int(duration / 0.5)
        for _ in range(steps):
            msg = String()
            msg.data = gait
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.5)

    node.get_logger().info("✅ Hoàn thành sequence. Đóng Webots để thoát.")
    node.destroy_node()
    rclpy.shutdown()


def start_gait_thread(context):
    t = threading.Thread(target=run_gait_sequence, daemon=True)
    t.start()
    return []


# ─────────────────────────────────────────────────────────────

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

    webots_driver = RosNode(
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

    robot_state_publisher = RosNode(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

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

        LogInfo(msg='[Sequence Mode] Webots đang khởi động...'),
        LogInfo(msg=f'[Sequence Mode] Gait sequence bắt đầu sau {STARTUP_DELAY}s'),

        webots_driver,
        robot_state_publisher,
        shutdown_handler,
        OpaqueFunction(function=start_gait_thread),
    ])