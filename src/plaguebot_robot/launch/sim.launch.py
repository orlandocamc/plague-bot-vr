from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, TimerAction,
    SetEnvironmentVariable, DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_robot = get_package_share_directory('plaguebot_robot')
    pkg_base  = get_package_share_directory('plaguebot_base_description')

    urdf_file  = os.path.join(pkg_robot, 'urdf', 'plaguebot_robot.urdf.xacro')
    world_file = os.path.join(pkg_robot, 'worlds', 'greenhouse.sdf')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' is_sim:=true']),
        value_type=str
    )

    # Expose GZ_SIM_RESOURCE_PATH so Gazebo can find installed package share dirs
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            os.path.join(os.path.expanduser('~'), 'plaguebot_ws', 'install',
                         'plaguebot_base_description', 'share'),
            os.path.join(os.path.expanduser('~'), 'plaguebot_ws', 'install',
                         'plaguebot_description', 'share'),
            os.path.join(os.path.expanduser('~'), 'plaguebot_ws', 'install',
                         'plaguebot_robot', 'share'),
        ])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # headless:=true runs Gazebo server-only (gz sim -s, no GUI), freeing the CPU
    # the GUI render eats. Useful when running Nav2 + RViz, where the control and
    # planner loops otherwise starve under the combined sim+GUI load.
    headless = LaunchConfiguration('headless')
    gz_args = PythonExpression([
        "'-r -s ' if '", headless, "' == 'true' else '-r '"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': [gz_args, world_file]}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'plaguebot',
            '-topic', 'robot_description',
            '-x', '0.5',
            '-y', '-5.0',
            '-z', '0.30',
            # Initial joint positions — arm stays at home until controller loads
            '-J', 'joint_1', '0',
            '-J', 'joint_2', '0',
            '-J', 'joint_3', '0',
            '-J', 'joint_4', '0',
            '-J', 'joint_5', '0',
            '-J', 'joint_6', '0',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/kiyo/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # The rgbd_camera (<topic>/d435</topic>) emits fixed gz suffixes
            # /d435/image, /d435/depth_image, /d435/points. Bridge those real gz
            # names, then remap below to the SPEC's ROS names (/d435/image_raw,
            # /d435/depth/image_raw, /d435/depth/points).
            '/d435/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/d435/image', '/d435/image_raw'),
            ('/d435/depth_image', '/d435/depth/image_raw'),
            ('/d435/points', '/d435/depth/points'),
        ],
        output='screen'
    )

    load_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    load_diff_drive = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active', 'diff_drive_controller'],
                output='screen'
            )
        ]
    )

    load_arm_controller = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active', 'arm_controller'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        load_joint_state_broadcaster,
        load_diff_drive,
        load_arm_controller,
    ])
