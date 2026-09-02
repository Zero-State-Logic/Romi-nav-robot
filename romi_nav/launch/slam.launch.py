"""
slam.launch.py
---------------
Runs slam_toolbox in "online async" mode to build a map while you
drive the robot around (e.g. via `ros2 run teleop_twist_keyboard
teleop_twist_keyboard` publishing to /cmd_vel, or by manually
publishing Twist messages).

Usage:
  1. In one terminal:  ros2 launch romi_nav bringup.launch.py
  2. In another:       ros2 launch romi_nav slam.launch.py
  3. Drive the robot around the world until slam_toolbox has mapped
     it (watch in RViz2 subscribed to /map, or check the terminal).
  4. Save the map with the standard tool:
       ros2 run nav2_map_server map_saver_cli -f ~/romi_ws/maps/romi_map
     This overwrites romi_map.pgm + romi_map.yaml, which is what
     nav.launch.py loads for static-map navigation.

This just includes slam_toolbox's own online_async_launch.py with
use_sim_time=true (required because Webots is our clock source, not
the wall clock) -- no need to reinvent slam_toolbox's launch logic.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Webots simulation clock instead of the wall clock'
        ),
        slam_toolbox_launch,
    ])
