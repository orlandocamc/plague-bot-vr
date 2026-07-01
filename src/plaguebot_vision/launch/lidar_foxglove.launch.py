#!/usr/bin/env python3
"""Launch RPLIDAR C1 driver + Foxglove bridge for remote visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    foxglove_port = LaunchConfiguration('foxglove_port', default='8765')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='USB serial port for RPLIDAR C1'),

        DeclareLaunchArgument(
            'foxglove_port',
            default_value=foxglove_port,
            description='WebSocket port for Foxglove bridge'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': foxglove_port,
                'address': '0.0.0.0',
                'tls': False,
                'topic_whitelist': ['.*'],
                'param_whitelist': ['.*'],
                'service_whitelist': ['.*'],
                'num_threads': 2,
            }],
            output='screen',
        ),
    ])
