import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('semih_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'semih.urdf.xacro')

    # Must be wrapped in ParameterValue(value_type=str): launch_ros otherwise tries to
    # parse the xacro-expanded URDF string as YAML and fails on stray colons/dashes
    # once plugins are added (see semiH_troubleshooting_log.md #4).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])