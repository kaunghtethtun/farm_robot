# Copyright (c) 2018 Intel Corporation
# Copyleft (🄯) 2025 ROM Robotics Corporation
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, LoadComposableNodes, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

#prefix = 'rom_'
prefix = ''


def generate_launch_description():
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    rom_robot_name = os.environ.get('ROM_ROBOT_MODEL', 'rom2109')
    
    bringup_dir = get_package_share_directory(f'{rom_robot_name}_nav2')

    remapping = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/diffbot_base_controller/odom', 'odom'),]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params]
            ),
            ComposableNode(
                package=f'{prefix}nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remapping + [('cmd_vel', 'cmd_vel_controller_server_to_vel_smoother')]),
            ComposableNode(
                package=f'{prefix}nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remapping),
           ComposableNode(
                package=f'{prefix}nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remapping),
            ComposableNode(
                package=f'{prefix}nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remapping+ [('cmd_vel_teleop', 'cmd_vel_teleop'), ('cmd_vel', 'cmd_vel_bhserver')]),
            ComposableNode(
                package=f'{prefix}nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remapping),
            ComposableNode(
                package=f'{prefix}nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remapping),
            ComposableNode(
                package=f'{prefix}nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remapping + [('cmd_vel', 'cmd_vel_controller_server_to_vel_smoother'), ('cmd_vel_smoothed', 'cmd_vel_smoother_to_collision')]),
            ComposableNode(
                package=f'{prefix}nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[configured_params],
                remappings=remapping+ [('cmd_vel', 'cmd_vel_smoother_to_collision'), ('cmd_vel', 'cmd_vel_collision_to_twist')]),
        ],
        output='screen'
    )
    
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,  # Automatically activate lifecycle nodes
            'node_names': ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']  # List of lifecycle nodes to manage
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_composition', default_value='True', description='Use composed bringup'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'), description='Default Path'),
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        container,
        lifecycle_manager_nav
    ])