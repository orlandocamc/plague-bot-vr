from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
        description="Si es True, apaga los nodos que Gazebo ya corre por su cuenta."
    )
    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command(
        [
            "xacro ",
            os.path.join(get_package_share_directory("plaguebot_description"), "urdf", "plaguebot.urdf.xacro")
        ]
        ),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim)
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("plaguebot_controller"),
                "config",
                "plaguebot_controllers.yaml"
            )
        ],
        condition=UnlessCondition(is_sim)
    )

    # Activa el hardware RobotSystem antes de lanzar los controladores.
    # Necesario en ros2_control >= 2.54.0 (Hardware Lifecycle Management).
    hardware_activator = Node(
        package="controller_manager",
        executable="hardware_spawner",
        arguments=[
            "RobotSystem",
            "--activate",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # arm_controller arranca solo después de que el hardware esté activo
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher,
        controller_manager,
        # Secuencia estricta: hardware → joint_state_broadcaster → arm_controller
        hardware_activator,
        RegisterEventHandler(
            OnProcessExit(
                target_action=hardware_activator,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner]
            )
        )
    ])
