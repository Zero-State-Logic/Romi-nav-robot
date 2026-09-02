"""
nav.launch.py
--------------
Loads the SAVED static map (~/romi_ws/maps/romi_map.yaml by default)
and starts AMCL + the full Nav2 stack against it. This does NOT run
SLAM -- localization only, against a pre-built map. Run
slam.launch.py separately if you need to (re)build the map first.

Prerequisite: bringup.launch.py must already be running in another
terminal (this launch file does not start Webots/the robot driver --
it only starts the navigation stack that talks to it over /scan,
/odom, /tf, and /cmd_vel).

Usage:
  ros2 launch romi_nav nav.launch.py
  ros2 launch romi_nav nav.launch.py map:=/path/to/other_map.yaml

This wraps nav2_bringup's own bringup_launch.py (map_server + amcl +
controller/planner/behavior servers + bt_navigator + lifecycle
manager) rather than reimplementing it -- we only override the
defaults for `map`, `params_file`, and `use_sim_time`, and pin
`slam:=False` since localization here is AMCL against a static map,
not live SLAM.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    romi_nav_dir = get_package_share_directory('romi_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map_path = os.path.expanduser('~/romi_ws/maps/romi_map.yaml')
    default_params_path = os.path.join(romi_nav_dir, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam': 'False',
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to the saved map YAML to localize/navigate against'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to the Nav2 parameters YAML'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Webots simulation clock instead of the wall clock'
        ),
        nav2_bringup_launch,
    ])
