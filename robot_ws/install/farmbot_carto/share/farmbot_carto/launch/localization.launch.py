import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    carto_pkg = get_package_share_directory('farmbot_carto')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(carto_pkg, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rom_nav_2d.lua')
    load_state_filename = LaunchConfiguration('load_state_filename', default='/home/mr_robot/data/maps/e.pbstream')

    # <include file="$(find reeman_localization)/launch/match_map.launch">

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # remappings=[('/odom', '/ekf_odom_local')],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', load_state_filename],
            ), # to add map.pbstream -> pgm -> map_server open ( how to save map by carto )

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
    
    ])
