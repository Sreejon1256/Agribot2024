from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=['/home/sreejon/Agribot2024/ros_ws/src/Agri/urdf/agribot.urdf'],
            output='screen',
        ),
    ])
