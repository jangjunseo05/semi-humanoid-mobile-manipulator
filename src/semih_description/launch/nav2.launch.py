import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('semih_description'), 'config', 'nav2_params.yaml'
    )
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('nav2_params_file'),
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items(),
        ),
    ])
