from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # backend: mock (sim default) | torch | ncnn | hailo  (see ADR-0003)
        DeclareLaunchArgument('backend', default_value='mock'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='plaguebot_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'backend': backend,
                'model_path': model_path,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
