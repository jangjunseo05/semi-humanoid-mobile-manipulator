"""
Single entry point that used to be 4-5 separate terminals: sim, SLAM, Nav2.
(YOLO / arm_reach / hdf5_collector are left as separate `ros2 run` commands
since you typically want to start/stop those independently while iterating.)

Usage:
    ros2 launch semih_description bringup.launch.py mode:=sim      # just Gazebo (default)
    ros2 launch semih_description bringup.launch.py mode:=slam     # + slam_toolbox
    ros2 launch semih_description bringup.launch.py mode:=nav      # + slam_toolbox + Nav2
    ros2 launch semih_description bringup.launch.py mode:=explore  # + slam_toolbox + Nav2 + explore_lite
    ros2 launch semih_description bringup.launch.py mode:=nav headless:=false

mode:=explore drives around and picks its own goals (frontier exploration, via the
explore_lite package under src/m-explore-ros2) -- no human-provided goal needed. It
stops on its own once there are no more reachable unexplored frontiers.

The robot spawns at exactly (0,0), which lands precisely on the edge of the initial
SLAM-derived costmap ("Robot is out of bounds of the costmap!" / "Sensor origin ...
is out of map bounds"), so explore_lite's first frontier search fails immediately and
gives up for good (it doesn't retry after "No frontiers found"). mode:=explore works
around this with a short automatic forward nudge 20s after startup -- enough to move
the robot's (x,y) off that exact boundary coordinate -- before explore_lite starts.
A pure rotation does NOT fix this (it changes orientation, not position); it has to
be linear motion.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('semih_description')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description="One of: sim, slam, nav, explore. Each mode includes everything the ones before it do.",
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description="Passed through to spawn_gazebo.launch.py -- see that file for why the default is true.",
    )
    mode = LaunchConfiguration('mode')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_gazebo.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )

    want_slam = IfCondition(PythonExpression(["'", mode, "' in ('slam', 'nav', 'explore')"]))
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam.launch.py')),
        condition=want_slam,
    )

    want_nav = IfCondition(PythonExpression(["'", mode, "' in ('nav', 'explore')"]))
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'nav2.launch.py')),
        condition=want_nav,
    )

    want_explore = IfCondition(PythonExpression(["'", mode, "' == 'explore'"]))

    # Nudge the robot off the exact spawn-point costmap-edge case described above.
    # Timed (rather than triggered off a topic) because it just needs to happen
    # sometime after slam_toolbox+nav2 are up and before explore_lite starts --
    # exact timing isn't critical.
    priming_spin = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            # Slow on purpose, but the duration needs real margin: `ros2 topic pub`
            # burns a chunk of its own timeout on discovery/handshake before the
            # first message even goes out, so a short timeout (tried 1.5s) produced
            # only ~3 actual velocity messages, not enough net displacement to clear
            # the boundary. 5s at 0.1m/s (~0.5m) is short enough not to run into
            # anything. The 3s of explicit zero-velocity afterward is deliberately
            # long/repeated -- a single "{}" publish wasn't enough to fully cancel
            # the motion in testing.
            cmd=['bash', '-c',
                 'timeout 5 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"; '
                 'timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"'],
            output='screen',
        )],
        condition=want_explore,
    )

    explore = TimerAction(
        period=33.0,
        actions=[IncludeLaunchDescription(
            # Our own wrapper, not explore_lite's bundled launch file -- see
            # launch/explore.launch.py's docstring for why.
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'explore.launch.py')),
        )],
        condition=want_explore,
    )

    return LaunchDescription([mode_arg, headless_arg, sim, slam, nav2, priming_spin, explore])
