from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='plaguebot_vision',
            namespace='',
            name='camera_realsense',
            executable='camera_realsense.py',
            output='screen'
        )
    ])
