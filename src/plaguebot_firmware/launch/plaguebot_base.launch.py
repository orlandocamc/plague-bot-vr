import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_path
import xacro

def generate_launch_description():
    urdf_path = os.path.join(
        str(get_package_share_path('plaguebot_base_description')),
        'urdf', 'plaguebot_base.urdf.xacro')
    robot_description = xacro.process_file(urdf_path, mappings={
        'is_sim': 'false'}).toxml()
    controllers_config = os.path.join(
        str(get_package_share_path('plaguebot_firmware')),
        'config', 'plaguebot_controllers.yaml')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_config],
        output='screen')
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen')
    plaguebot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['plaguebot_controller'],
        output='screen')
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=5.0, actions=[plaguebot_controller]),
    ])
