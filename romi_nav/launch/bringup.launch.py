"""
bringup.launch.py
------------------
Starts the Webots simulator loaded with ROMI Lab's custom world
(worlds/romi_small.wbt) and connects the TurtleBot3 Burger's ROS 2
"extern" controller to it, plus the Ros2Supervisor and
robot_state_publisher. This is a direct adaptation of the
webots_ros2_turtlebot demo launch file (robot_launch.py), with two
changes: (1) it points at OUR world file instead of the bundled
apartment demo world, and (2) it drops the demo's built-in
turtlebot3_navigation2 / turtlebot3_cartographer auto-includes --
those are handled separately by slam.launch.py and nav.launch.py
in this package, kept deliberately independent so you can bring up
the robot on its own without SLAM or Nav2 running.

How the "extern controller" connection actually works (read this if
you ever see "controller not connecting" errors):
  1. WebotsLauncher starts the Webots process itself and, because
     ros2_supervisor=True, also starts the Ros2Supervisor extern
     controller process (the Robot{name "Ros2Supervisor"} node in
     the .wbt file has controller "<extern>", meaning Webots does
     NOT run its own controller code for it -- it just waits for
     an external process to attach to it over Webots' inter-process
     link).
  2. WebotsController (the `turtlebot_driver` node below) is that
     same kind of external process, but for the TurtleBot3Burger
     robot node (which also has controller "<extern>" in the world
     file). It attaches by matching `robot_name` to the Robot node's
     name field in the world.
  3. Both of those handshakes take a few seconds after Webots opens.
     WaitForControllerConnection blocks starting the ros2_control
     spawners (diffdrive_controller, joint_state_broadcaster) until
     `turtlebot_driver` reports it is actually connected -- this is
     what prevents the classic race condition where the spawners try
     to talk to a controller_manager that isn't live yet.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    # This package's own share dir -- holds OUR world file.
    romi_nav_dir = get_package_share_directory('romi_nav')
    # The stock TurtleBot3 package's share dir -- holds the generic
    # robot URDF and ros2_control config, which are properties of the
    # *robot model*, not of our world, so we reuse them as-is.
    turtlebot_pkg_dir = get_package_share_directory('webots_ros2_turtlebot')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([romi_nav_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # Placeholder robot_state_publisher: the *real* robot_description
    # (full URDF) is pushed to robot_state_publisher by the
    # WebotsController driver below via set_robot_state_publisher=True.
    # This node here just exists early so TF has a publisher available
    # immediately, matching the demo launch exactly.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(turtlebot_pkg_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(turtlebot_pkg_dir, 'resource', 'ros2control.yml')

    # Humble publishes plain Twist on cmd_vel_unstamped; only
    # rolling/jazzy switched to TwistStamped. We're pinned to Humble,
    # but keep the same check as upstream in case that ever changes.
    use_twist_stamped = os.environ.get('ROS_DISTRO') in ['rolling', 'jazzy']
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')]

    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Gate: don't spawn the ros2_control controllers until Webots has
    # actually connected to turtlebot_driver. This is the fix for the
    # "extern controller not connecting" race mentioned in the task.
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='romi_small.wbt',
            description='World file (relative to romi_nav/worlds) to load in Webots'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode (realtime, fast, pause)'
        ),

        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        turtlebot_driver,
        waiting_nodes,

        # Bring the whole launch down cleanly when the Webots window closes.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
