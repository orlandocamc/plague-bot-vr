import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_loc = get_package_share_directory('plaguebot_localization')
    ekf_params = os.path.join(pkg_loc, 'config', 'ekf.yaml')

    # EKF fusing wheel odometry (/diff_drive_controller/odom) and IMU (/imu/data).
    # Publishes the odom->base_footprint TF that diff_drive_controller used to
    # publish (now disabled via enable_odom_tf: false in robot_controllers.yaml).
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    return LaunchDescription([ekf])
