"""Full mission stack (SPEC §6.2).

Brings up: nav.launch.py (sim + EKF + Nav2) + perception_node + mission_node +
rosbridge_websocket (for the VR page) + a static HTTP server for the WebXR page.

move_group is optional (use_moveit:=true) and only needed for the IK_POSITION
step; the rest of the mission runs without it (see mission_node docstring).
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('plaguebot_nav')
    pkg_perception = get_package_share_directory('plaguebot_perception')
    pkg_mission = get_package_share_directory('plaguebot_mission')

    web_dir = os.path.join(pkg_mission, 'web')

    backend = LaunchConfiguration('backend')
    headless = LaunchConfiguration('headless')
    use_moveit = LaunchConfiguration('use_moveit')

    declared = [
        DeclareLaunchArgument('backend', default_value='mock'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_moveit', default_value='false'),
    ]

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py')),
        launch_arguments={'backend': backend}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('plaguebot_moveit'),
                         'launch', 'move_group.launch.py')),
        condition=IfCondition(use_moveit),
    )

    mission = Node(
        package='plaguebot_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # VR interface: rosbridge WebSocket on :9090 (SPEC §6.1).
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
    )

    # Serve the WebXR page so the Quest 3 browser can load it from the robot.
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', web_dir],
        output='screen',
    )

    # Give the sim + Nav2 time to come up before mission/perception attach.
    delayed = TimerAction(
        period=12.0,
        actions=[perception, move_group, mission, rosbridge, web_server],
    )

    return LaunchDescription(declared + [nav, delayed])
