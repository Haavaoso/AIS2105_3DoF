from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    urdf_dir = '/home/marianne/workspaces/ros2_ws/src/almostFinish/urdf'
    rviz_dir = '/home/marianne/workspaces/ros2_ws/src/almostFinish/rviz'
    
    urdf_file = PathJoinSubstitution([urdf_dir, '3dof_platform.xacro'])
    rviz_config_file = PathJoinSubstitution([rviz_dir, 'display.rviz'])
    robot_description_content = Command(['xacro ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
            name='robot_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])

