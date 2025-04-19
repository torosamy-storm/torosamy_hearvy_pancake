import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, EmitEvent, RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals,IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

torosamy_navigation_server_launcher_dir = get_package_share_directory('torosamy_navigation_server_launcher')
def generate_launch_description():
    ld = LaunchDescription()
    arguments = [
        DeclareLaunchArgument('rviz',default_value='True'),
        DeclareLaunchArgument('use_sim_time',default_value='False'),
        DeclareLaunchArgument('map',default_value='base'),
        DeclareLaunchArgument('mode',default_value='nav'),
        DeclareLaunchArgument('namespace',default_value=''),
        DeclareLaunchArgument('localization',default_value='slam'),
        DeclareLaunchArgument('robot',default_value='polar_bear_robot_real'),
    ]
    for argument in arguments:
        ld.add_action(argument)



    ld.add_action(publish_robot_state())
    ld.add_action(start_livox_ros_driver2())
    ld.add_action(start_linefit_ground_segmentation())
    ld.add_action(start_pointcloud_to_laserscan())
    ld.add_action(start_point_lio())
    ld.add_action(start_localization())
    ld.add_action(start_mapping())
    ld.add_action(start_navigation())
    ld.add_action(start_rviz())

    return ld

def start_navigation()->IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(torosamy_navigation_server_launcher_dir,'launch', 'start_navigation.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'namespace': LaunchConfiguration('namespace')
        }.items()
    )


def start_rviz()->GroupAction:
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('torosamy_navigation_server_launcher'), 'config', 'navigation.rviz')],
        output='screen'
    )

    return GroupAction(
        condition = IfCondition(LaunchConfiguration('rviz')),
        actions=[
            start_rviz_cmd,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=start_rviz_cmd,
                    on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
                )
            )
        ]
    )

def start_linefit_ground_segmentation()->Node:
    return Node(
        package='linefit_ground_segmentation',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[
            os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'segmentation.yaml')
        ]
    )
def start_pointcloud_to_laserscan()->Node:
    return Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in',  ['/segmentation/obstacle']),
            ('scan',  ['/scan'])
        ],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01, #0.01
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )
def start_point_lio()->GroupAction:
    return GroupAction(
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    '--frame-id', 'odom',
                    '--child-frame-id', 'lidar_odom'
                ],
            ),
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='laserMapping',
                output='screen',
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                parameters=[
                    os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'mid360.yaml')
                ],
            )
        ]
    )


def start_mapping()->Node:
    return Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'mapping.yaml')
        ],
    )

def publish_robot_state()->Node:
    robot_description = Command([
        'xacro ', PathJoinSubstitution([torosamy_navigation_server_launcher_dir, 'urdf', LaunchConfiguration('robot'),"robot.urdf.xacro"]),
        ' xyz:="0.0 0.0395 0.5074" rpy:="0.0 0.0 0.0"'
    ])

    # launch_params = yaml.safe_load(open(os.path.join(torosamy_navigation_server_launcher_dir, 'config', 'measurement.yaml')))
    

    # robot_description = Command([
    #     'xacro ', PathJoinSubstitution([torosamy_navigation_server_launcher_dir, 'urdf', LaunchConfiguration('robot'),"robot.urdf.xacro"]),
    #     ' xyz:=', launch_params['base_link2livox_frame']['real']['xyz'], 
    #     ' rpy:=', launch_params['base_link2livox_frame']['real']['rpy']
    # ])


    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description
        }],
        output='screen',
        respawn=True
    )



def start_livox_ros_driver2()-> Node:
    return Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        parameters=[
            {
                "xfer_format": 4,  # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
                "multi_topic": 0,  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
                "data_src": 0,  # 0-lidar, others-Invalid data src
                "publish_freq": 10.0,  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "cmdline_input_bd_code": "livox0000000001",
                "user_config_path": os.path.join(torosamy_navigation_server_launcher_dir,'config', 'MID360.json'),
            }
        ],
    )


def start_localization()->IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(torosamy_navigation_server_launcher_dir,'launch','start_localization.py')),
        condition = LaunchConfigurationEquals('mode', 'nav'),
        launch_arguments = {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'localization': LaunchConfiguration('localization')
        }.items()
    )
