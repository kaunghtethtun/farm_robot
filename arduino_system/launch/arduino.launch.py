from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            executable='serial_interface_node',
            package='arduino_system',
        ),
        Node(
            executable='odom_node',
            package='arduino_system',
        ),
    ])