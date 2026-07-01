from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='plaguebot_vision',
            namespace='',
            name='camera_kiyo',
            executable='camera_kiyo.py',
            output='screen'
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='plaguebot_vision',
                    namespace='',
                    name='camera_realsense',
                    executable='camera_realsense.py',
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='plaguebot_vision',
                    namespace='',
                    name='webrtc_server',
                    executable='webrtc_server.py',
                    output='screen'
                ),
            ]
        ),
    ])
