import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('kiyocore_bringup'),
        'config', 'ekf.yaml')

    return LaunchDescription([

        # 1. micro-ROS agent (ESP32 WROOM motor controller)
        ExecuteProcess(
            cmd=['bash', '-c',
                 'source /opt/ros/humble/setup.bash && '
                 'source /home/ubuntu/microros_ws/install/local_setup.bash && '
                 'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200'],
            output='screen'
        ),

        # 2. LiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
            output='screen'
        ),

        # 3. IMU node
        ExecuteProcess(
            cmd=['python3', '/home/ubuntu/imu_node.py'],
            output='screen'
        ),

        # 4. base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_laser',
            arguments=['0', '0', '0.20', '0', '0', '0', 'base_link', 'laser']
        ),

        # 5. base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_imu',
            arguments=['0.04', '-0.02', '0.075', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # 6. rf2o laser odometry (publish_tf False; EKF owns odom->base_link)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rf2o_laser_odometry',
                    executable='rf2o_laser_odometry_node',
                    name='rf2o_laser_odometry',
                    output='screen',
                    parameters=[{
                        'laser_scan_topic': '/scan',
                        'odom_topic': '/odom_rf2o',
                        'publish_tf': False,
                        'base_frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'init_pose_from_topic': '',
                        'freq': 20.0,
                    }],
                ),
            ]
        ),

        # 7. EKF fusion (rf2o + IMU) -> odom->base_link
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    parameters=[ekf_config],
                    output='screen'
                ),
            ]
        ),
    ])
