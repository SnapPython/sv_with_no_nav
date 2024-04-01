import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch.actions import DeclareLaunchArgument

import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetUseSimTime

from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml

livox_path = get_package_share_directory('livox_ros_driver2')

################### user configure parameters for ros2 start ###################
xfer_format   = 4    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 20.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = '47MDL330010038'

cur_config_path = os.path.join(livox_path, 'config')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rm_bringup').find('rm_bringup')
    default_model_path = os.path.join(pkg_share, 'src/urdf/test.urdf')

    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config', 'mid360.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    # lidar_tf = Node(
    #     name='lidar_tf',
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0','0','-0.55','0','0','0','1','body','base_link']
    #     )

    ld = LaunchDescription()
    #ld.add_action(declare_use_sim_time_cmd)
    #ld.add_action(declare_config_path_cmd)
    #ld.add_action(declare_rviz_cmd)
    #ld.add_action(declare_rviz_config_path_cmd)

    #ld.add_action(fast_lio_node)
    # ld.add_action(lidar_tf)
    # ld.add_action(rviz_node)

    imu_node = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ],
                remappings=[
                	('/imu/data_raw', '/livox/imu'),
                ]
            )
    
    pointcloud_node = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  ['/segmentation/obstacle']),
                        ('scan',  ['/scan'])],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': 0.1,
                'angle_min': -3.14159,# -M_PI/2 #0.39269875
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.4,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    
    scanner = DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        )
    
    slam_params_file = os.path.join(get_package_share_directory("rm_navigation"),
                                   'params', 'mapper_params_online_async.yaml'),

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rm_navigation"),
                                   'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    #ld.add_action(declare_use_sim_time_argument)
    #ld.add_action(declare_slam_params_file_cmd)
    #ld.add_action(start_async_slam_toolbox_node)

    # Get the launch directory
    bringup_dir = get_package_share_directory('rm_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value= os.path.join(bringup_dir,'map','map_1705367110.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
        #     condition=IfCondition(slam),
        #     launch_arguments={'namespace': namespace,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'use_respawn': use_respawn,
        #                       'params_file': params_file}.items()),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir,
        #                                                'localization_launch.py')),
        #     condition=IfCondition(PythonExpression(['not ', slam])),
        #     launch_arguments={'namespace': namespace,
        #                       'map': map_yaml_file,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': params_file,
        #                       'use_composition': use_composition,
        #                       'use_respawn': use_respawn,
        #                       'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py'))
        ),                      
    ])

    # Create the launch description and populate
    # Set environment variables
    # ld.add_action(stdout_linebuf_envvar)

    # # Declare the launch options
    # ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_use_namespace_cmd)
    # ld.add_action(declare_slam_cmd)
    # ld.add_action(declare_map_yaml_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_autostart_cmd)
    # ld.add_action(declare_use_composition_cmd)
    # ld.add_action(declare_use_respawn_cmd)
    # ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    node_params = os.path.join(
        get_package_share_directory('rm_decision'), 'config', 'node_params.yaml')
    
    rm_decision_node = Node(
        package="rm_decision",
        name="rm_decision_node",
        executable="rm_decision_node",
        namespace="",
        output="screen",
        parameters=[node_params],
    )

    # Getting directories and launch-files
    bringup_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    params_file = os.path.join(bringup_dir, 'launch', 'segmentation_params.yaml')

    # Nodes launching commands
    node_start_cmd = Node(
            package='linefit_ground_segmentation_ros',
            executable='ground_segmentation_node',
            output='screen',
            parameters=[params_file])

    # Declare the launch options
    ld.add_action(node_start_cmd)

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        livox_driver,
        pointcloud_node,
        imu_node,
        scanner,
        rm_decision_node,
        ld,
    ])

