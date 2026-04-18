from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
        ),
    ])
