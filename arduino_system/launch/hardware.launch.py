#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arduino_system'),
                'launch',
                'rplidar_a2.launch.py'
            )
        )
    )

    nodes = [
        description_launch,
        rplidar_launch,
        arduino_launch,
    ]

    return LaunchDescription(nodes)
