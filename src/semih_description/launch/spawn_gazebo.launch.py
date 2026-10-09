import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('semih_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'semih.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'semih_world.sdf')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str
    )

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    headless = LaunchConfiguration('headless').perform(context) == 'true'
    # -s runs the Gazebo server without the GUI client. On WSL2 this matters a lot:
    # Gazebo's ogre2 renderer commonly falls back to software rendering (llvmpipe) here,
    # and the GUI client runs a second full render pipeline on top of the server's own
    # sensor rendering (camera/depth/lidar), which is what drove CPU to 800%+ in past runs.
    gz_flags = '-s -r' if headless else '-r'
    gz_args = f'{world_path} {gz_flags}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'semih',
            '-z', '0.2',
        ],
        output='screen',
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    # Without this, every use_sim_time:=true node (robot_state_publisher, slam_toolbox,
    # all of Nav2) has its ROS clock stuck at 0 forever, since nothing publishes /clock.
    # Nav2's costmap update loop is a sim-time timer, so it never fires again after the
    # very first tick -- the robot looks like it's "driving" (BT cycles RUNNING/SUCCESS
    # in microseconds) but /cmd_vel stays zero forever because the costmaps never update.
    # See CLAUDE.md section A-5 for the full diagnosis.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    joint_state_bridge_config = os.path.join(pkg_share, 'config', 'joint_state_bridge.yaml')
    wheel_joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': joint_state_bridge_config}],
        output='screen',
    )

    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'semih/base_link/lidar'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    return [
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        cmd_vel_bridge,
        clock_bridge,
        camera_bridge,
        camera_info_bridge,
        lidar_bridge,
        depth_bridge,
        wheel_joint_state_bridge,
        lidar_frame_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description=(
            "Run Gazebo without the GUI client ('-s'). Default true: on WSL2 the ogre2 "
            'renderer commonly falls back to CPU software rendering, and running the GUI '
            'on top of that has been the main driver of CPU overload in this project '
            "(see semiH_troubleshooting_log.md #16). Pass headless:=false for visual debugging."
        ),
    )
    return LaunchDescription([
        headless_arg,
        OpaqueFunction(function=launch_setup),
    ])
