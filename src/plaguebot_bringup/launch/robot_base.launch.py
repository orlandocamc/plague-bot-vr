"""Real-robot base bringup: URDF + esp32_bridge + EKF + RPLIDAR C1.

No Gazebo, no ros2_control — the ESP32 bridge owns motion and publishes /odom
and /imu/data; robot_localization's EKF fuses them and owns the
odom->base_footprint TF. slam_toolbox / AMCL provide map->odom on top.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("plaguebot_bringup")
    sllidar = get_package_share_directory("sllidar_ros2")

    xacro_file = os.path.join(pkg, "urdf", "plaguebot_base.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " is_sim:=false"]), value_type=str
    )

    use_lidar = LaunchConfiguration("use_lidar")

    return LaunchDescription([
        DeclareLaunchArgument("use_lidar", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": False}],
        ),
        # Publishes zero states for the continuous wheel joints so the TF tree
        # is complete (esp32_bridge does not emit /joint_states).
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"use_sim_time": False}],
        ),
        Node(
            package="plaguebot_bringup",
            executable="esp32_bridge",
            name="esp32_bridge",
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(pkg, "config", "ekf.yaml")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar, "launch", "sllidar_c1_launch.py")
            ),
            condition=IfCondition(use_lidar),
        ),
    ])
