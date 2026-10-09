import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('semih_description'), 'config', 'slam_params.yaml'
    )
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
