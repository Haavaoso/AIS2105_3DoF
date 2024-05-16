from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("node_hell"), "urdf", "arduino_driver.urdf.xacro"]),  # Updated to node_hell
            " ",
            "prefix:=",
            "ard_",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("node_hell"), "config", "arduino_controllers.yaml"]  # Updated to node_hell
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"]
    )

    follow_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["follow_joint_trajectory"]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        controller_spawner,
        trajectory_spawner,
        follow_trajectory_spawner
    ]

    return LaunchDescription(nodes_to_start)

