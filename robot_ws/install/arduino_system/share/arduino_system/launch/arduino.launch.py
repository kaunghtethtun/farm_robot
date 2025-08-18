from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_system',  # Replace with your package name
            executable='odom',       # This should match your add_executable() in CMakeLists.txt
            name='odom',
            output='screen',
            parameters=[
                {
                    'wheel_radius': 0.04,
                    'wheel_base': 0.21,
                    'ticks_per_rev': 1317,
                    'publish_rate': 20.0,
                    'pub_tf': True,
                    'rpm_alpha': 0.5,
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint'
                }
            ],
            remappings=[
                ('encoder_counts', '/encoder_counts'),  # Adjust topic names if needed
                ('wheel_rpm', '/wheel_rpm')
            ]
        ),
        Node(
            package='arduino_system',
            executable='diff_controller',
        ),
        
    ])
