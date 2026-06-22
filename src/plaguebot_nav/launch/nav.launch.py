import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_robot = get_package_share_directory('plaguebot_robot')
    pkg_loc   = get_package_share_directory('plaguebot_localization')
    pkg_slam  = get_package_share_directory('plaguebot_slam')
    pkg_nav   = get_package_share_directory('plaguebot_nav')
    pkg_nav2  = get_package_share_directory('nav2_bringup')

    default_map    = os.path.join(pkg_slam, 'maps', 'greenhouse.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_config    = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    map_yaml     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')
    headless     = LaunchConfiguration('headless')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        # headless:=true -> Gazebo runs without its GUI, freeing CPU so the Nav2
        # control/planner loops don't starve under combined sim + RViz load.
        DeclareLaunchArgument('headless', default_value='false'),
    ]

    # Gazebo + robot + controllers + sensor bridge.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'headless': headless}.items()
    )

    # EKF: fuses wheel odom + IMU, publishes odom->base_footprint (the Nav2 prior).
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, 'launch', 'localization.launch.py')
        )
    )

    # Nav2 localization (map_server + AMCL): provides map->odom against the saved map.
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    # Nav2 navigation stack: planner, controller (MPPI), costmaps, BT, recoveries,
    # smoother, collision_monitor (which outputs to /diff_drive_controller/cmd_vel).
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    # Let the sim spawn the robot and load controllers (and the EKF start producing
    # odom->base_footprint) before Nav2 brings up AMCL and the costmaps.
    delayed_nav2 = TimerAction(
        period=10.0,
        actions=[nav2_localization, nav2_navigation],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription(
        declare_args + [sim, localization, delayed_nav2, rviz]
    )
