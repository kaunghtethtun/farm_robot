#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('farmbot_description'),
                'launch',
                'view_robot.launch.py'
            )
        )
    )

    arduino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arduino_system'),
                'launch',
                'arduino.launch.py'
            )
        )
    )
    
    twist_mux_params = os.path.join(get_package_share_directory('arduino_system'), 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_controller/cmd_vel_unstamped')]
    )

    imu_node = Node(
        package="wit_motion_imu_publisher",
        executable="imu",
        name="imu",
        condition=IfCondition(LaunchConfiguration('use_imu'))
    )

    static_tf_pub = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            condition=IfCondition(LaunchConfiguration('use_imu')),
            arguments=['0.000062039', '0.000055952', '0.004',    # Translation: x=1.0, y=2.0, z=0.0
                       '0.0', '0.0', '0.0', '1.0',  # Rotation: Quaternion (qx, qy, qz, qw)
                       'base_link', 'imu'],  # Parent frame and child frame
     )
    
    nodes = [
        description_launch,
        imu_node,
        twist_mux_node,
        static_tf_pub,
        arduino_launch,
    ]

    return LaunchDescription(nodes)
