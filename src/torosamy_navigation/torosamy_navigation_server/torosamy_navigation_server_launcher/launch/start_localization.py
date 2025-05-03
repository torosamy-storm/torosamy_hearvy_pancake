import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes,Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
torosamy_navigation_server_launcher_dir = get_package_share_directory('torosamy_navigation_server_launcher')


def generate_launch_description():
    ld = LaunchDescription()
    arguments = [
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('localization'),
        DeclareLaunchArgument('use_sim_time',default_value='False'),
        DeclareLaunchArgument('namespace',default_value=''),
        DeclareLaunchArgument('params_file',default_value=os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'navigation.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('container_name', default_value='nav2_container')
    ]

    for arg in arguments:
        ld.add_action(arg)

    ld.add_action(localzation_null())
    ld.add_action(localization_slam())
    # ld.add_action(localzation_gicp())
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'lidar_odom'
        ]
    ))
    return ld



def localzation_gicp()->LoadComposableNodes:
    container_name_full = (LaunchConfiguration('namespace'), '/', LaunchConfiguration('container_name'))
    remappings = [('/tf', 'tf'),('/tf_static', 'tf_static')]

    return LoadComposableNodes(
        condition = LaunchConfigurationEquals('localization', 'gicp'),
        target_container=container_name_full,
        composable_node_descriptions = [
            ComposableNode(
                package="small_gicp_relocalization",
                plugin="small_gicp_relocalization::SmallGicpRelocalizationNode",
                name="small_gicp_relocalization",
                parameters=[{
                    #"prior_pcd_file": PathJoinSubstitution([torosamy_navigation_server_launcher_dir, 'PCD', LaunchConfiguration('map'),".pcd"])
                    "prior_pcd_file": "/home/ubuntu/workspace/project/pcd2pgm/PCD/scans.pcd",
                    "use_sim_time": False,
                    "yaml_filename": PathJoinSubstitution([torosamy_navigation_server_launcher_dir, 'maps', LaunchConfiguration('map'),"map.yaml"])  
                }],
                remappings=remappings,
            )
        ]
    )


def localization_slam()->Node:
    return Node(
        condition = LaunchConfigurationEquals('localization', 'slam'),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'slam_toolbox_relocalization.yaml'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': PathJoinSubstitution([torosamy_navigation_server_launcher_dir, 'maps', LaunchConfiguration('map'), 'map']),
            }
        ]
    )
def localzation_null()->Node:
    return Node(
        condition = LaunchConfigurationEquals('localization', 'null'),
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ]
    )
