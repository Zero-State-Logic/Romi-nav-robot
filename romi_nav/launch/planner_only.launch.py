import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('romi_nav'), 'config', 'planner.yaml')
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[cfg]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planner', output='screen',
             parameters=[{'autostart': True, 'node_names': ['planner_server']}]),
    ])
