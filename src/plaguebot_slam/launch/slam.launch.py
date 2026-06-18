import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, EmitEvent, RegisterEventHandler, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam  = get_package_share_directory('plaguebot_slam')
    pkg_robot = get_package_share_directory('plaguebot_robot')
    pkg_loc   = get_package_share_directory('plaguebot_localization')

    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_slam, 'config', 'rviz_slam.rviz')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'sim.launch.py')
        )
    )

    # EKF state estimator: fuses wheel odom + IMU and publishes odom->base_footprint.
    # SLAM consumes this fused TF as its prior instead of raw wheel odometry.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, 'launch', 'localization.launch.py')
        )
    )

    # In Jazzy, async_slam_toolbox_node is a managed (lifecycle) node. It starts
    # UNCONFIGURED and only declares params / subscribes to /scan / publishes the
    # map->odom TF once transitioned to CONFIGURE then ACTIVATE. Launching it as a
    # plain Node leaves it inert. Replicate slam_toolbox's online_async_launch.py.
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam_toolbox] configured -> activating'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([sim, localization, slam, configure_slam, activate_slam, rviz])
