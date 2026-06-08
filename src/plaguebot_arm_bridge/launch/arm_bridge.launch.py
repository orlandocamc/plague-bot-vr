from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('plaguebot_arm_bridge'),
        'config',
        'arm_bridge.yaml'
    )

    bridge_node = Node(
        package='plaguebot_arm_bridge',
        executable='bridge_node',
        name='plaguebot_arm_bridge',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([bridge_node])
