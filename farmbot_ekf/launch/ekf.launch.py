from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # rom_robot_name = os.environ.get('ROM_ROBOT_MODEL', 'bobo')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('farmbot_ekf'), 'config', 'ekf.yaml')],
            remappings=[('cmd_vel', 'diff_controller/cmd_vel_unstamped'),('odometry/filtered' , 'odom' ), ('set_pose' , 'initialpose')],
           ),

])
