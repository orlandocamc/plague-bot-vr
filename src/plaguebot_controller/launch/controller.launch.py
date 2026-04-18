from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Creamos el "switch" para saber si estamos en simulación
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

    # 2. Le decimos a este nodo que NO arranque si estamos en simulación
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim)
    )

    # 3. Hacemos lo mismo con el cerebro de los motores físicos
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

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
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])