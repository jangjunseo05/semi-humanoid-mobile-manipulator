"""
Wraps explore_lite's node directly (rather than its bundled explore.launch.py)
because that launch file hardcodes its own package's default params.yaml with
no launch argument to override it. We need our own config -- see
config/explore_params.yaml for why (costmap_topic pointed at Nav2's global
costmap instead of the raw slam_toolbox map).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('semih_description'), 'config', 'explore_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('explore_params_file', default_value=default_params),
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[LaunchConfiguration('explore_params_file'), {'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
    ])
