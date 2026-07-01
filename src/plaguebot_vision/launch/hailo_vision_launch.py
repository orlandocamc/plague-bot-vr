"""Fase 2: grafo de visión sobre Hailo-8.

camera (webcam o realsense) -> hailo_detector -> vision_dashboard (MJPEG :8080).

Selecciona la cámara con el argumento `camera`:
  ros2 launch plaguebot_vision hailo_vision_launch.py camera:=webcam
  ros2 launch plaguebot_vision hailo_vision_launch.py camera:=realsense
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration('camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='webcam',
            description='Fuente de color: webcam | realsense'),

        Node(
            package='plaguebot_vision',
            executable='camera_webcam.py',
            name='camera_webcam',
            output='screen',
            condition=LaunchConfigurationEquals('camera', 'webcam'),
        ),
        Node(
            package='plaguebot_vision',
            executable='camera_realsense.py',
            name='camera_realsense',
            output='screen',
            condition=LaunchConfigurationEquals('camera', 'realsense'),
        ),

        # Segunda cámara USB del demo: independiente de `camera:=`, siempre activa.
        Node(
            package='plaguebot_vision',
            executable='camera_kiyo.py',
            name='camera_kiyo',
            output='screen',
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='plaguebot_vision',
                executable='hailo_detector.py',
                name='hailo_detector',
                output='screen',
            ),
        ]),

        TimerAction(period=3.0, actions=[
            Node(
                package='plaguebot_vision',
                executable='vision_dashboard.py',
                name='vision_dashboard',
                output='screen',
            ),
        ]),
    ])
