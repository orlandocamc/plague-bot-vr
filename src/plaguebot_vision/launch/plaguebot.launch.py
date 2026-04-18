from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('color_topic', default_value='/color/image_raw'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('prefer_h264', default_value='true'),
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='camera',
            parameters=[{
                'camera_name': 'camera',
                'enable_color': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'enable_depth': True,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
                'depth_format': 'Y11',
                'enable_ir': True,
                'ir_width': 640,
                'ir_height': 480,
                'ir_fps': 30,
                'ir_format': 'Y10',
            }],
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ),
        Node(
            package='plaguebot_vision',
            executable='webrtc_bridge',
            name='webrtc_bridge',
            parameters=[{
                'color_topic': LaunchConfiguration('color_topic'),
                'http_port': LaunchConfiguration('http_port'),
                'prefer_h264': LaunchConfiguration('prefer_h264'),
            }],
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ),
    ])
