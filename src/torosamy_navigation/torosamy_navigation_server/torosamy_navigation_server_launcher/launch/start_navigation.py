import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LoadComposableNodes
torosamy_navigation_server_launcher_dir = get_package_share_directory('torosamy_navigation_server_launcher')
def generate_launch_description():
    ld = LaunchDescription()
    arguments = [
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('use_slam',default_value='True'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time',default_value='False'),
        DeclareLaunchArgument('params_file',default_value=os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'navigation.yaml')),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('use_respawn', default_value='True'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
    ]
    for argument in arguments:
        ld.add_action(argument)




    ld.add_action(start_container_components())
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(start_navigation_server())


    return ld

def start_container_components()->Node:
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        convert_types=True,
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': PathJoinSubstitution([torosamy_navigation_server_launcher_dir, 'maps', LaunchConfiguration('map'),"map.yaml"])
        },
    )
    return Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[configured_params, {'autostart': LaunchConfiguration('autostart')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings,
        output='screen'
    )

def start_navigation_server()->LoadComposableNodes:
    

    container_name_full = (
        LaunchConfiguration('namespace'), 
        '/',
        LaunchConfiguration('container_name')
    )


    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']


    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]



    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        convert_types=True,
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart')
        }
    )

   

    return LoadComposableNodes(
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': lifecycle_nodes
                }]
            )
        ],
    )