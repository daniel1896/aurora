#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ----------------------- Launch Arguments -----------------------
    namespace = LaunchConfiguration('namespace')
    use_mavros = LaunchConfiguration('use_mavros')
    use_livox = LaunchConfiguration('use_livox')
    use_zed = LaunchConfiguration('use_zed')
    use_fastlio = LaunchConfiguration('use_fastlio')
    use_rviz = LaunchConfiguration('use_rviz')
    record_all = LaunchConfiguration('record_all')

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    livox_config = LaunchConfiguration('livox_config')
    fastlio_config = LaunchConfiguration('fastlio_config')

    zed_model = LaunchConfiguration('zed_model')
    zed_resolution = LaunchConfiguration('zed_resolution')
    zed_fps = LaunchConfiguration('zed_fps')
    zed_depth_mode = LaunchConfiguration('zed_depth_mode')

    # Frame unification
    lio_ref_frame = LaunchConfiguration('lio_ref_frame')     # usually 'camera_init' from FAST-LIO2
    publish_odom_identity = LaunchConfiguration('publish_odom_identity')  # publish odom->lio_ref_frame identity
    publish_map_identity = LaunchConfiguration('publish_map_identity')    # publish map->odom identity

    # Static TFs for sensors relative to base_link (adjust to your rig!)
    bl2livox_x = LaunchConfiguration('bl2livox_x')
    bl2livox_y = LaunchConfiguration('bl2livox_y')
    bl2livox_z = LaunchConfiguration('bl2livox_z')
    bl2livox_roll = LaunchConfiguration('bl2livox_roll')
    bl2livox_pitch = LaunchConfiguration('bl2livox_pitch')
    bl2livox_yaw = LaunchConfiguration('bl2livox_yaw')

    bl2zed_x = LaunchConfiguration('bl2zed_x')
    bl2zed_y = LaunchConfiguration('bl2zed_y')
    bl2zed_z = LaunchConfiguration('bl2zed_z')
    bl2zed_roll = LaunchConfiguration('bl2zed_roll')
    bl2zed_pitch = LaunchConfiguration('bl2zed_pitch')
    bl2zed_yaw = LaunchConfiguration('bl2zed_yaw')

    rviz_config = LaunchConfiguration('rviz_config')

    declares = [
        DeclareLaunchArgument('namespace', default_value=TextSubstitution(text=''), description='Top-level namespace'),
        DeclareLaunchArgument('use_mavros', default_value='true'),
        DeclareLaunchArgument('use_livox', default_value='true'),
        DeclareLaunchArgument('use_zed', default_value='true'),
        DeclareLaunchArgument('use_fastlio', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('record_all', default_value='false', description='Record all topics with rosbag2'),

        DeclareLaunchArgument('fcu_url', default_value=TextSubstitution(text='/dev/ttyTHS1:921600'), description='Serial for Pixhawk (MAVROS)'),
        DeclareLaunchArgument('gcs_url', default_value=TextSubstitution(text=''), description='Optional GCS endpoint'),

        DeclareLaunchArgument('livox_config', default_value=PathJoinSubstitution([FindPackageShare('aurora_bringup'), 'config', 'livox', 'mid70.json'])),
        DeclareLaunchArgument('fastlio_config', default_value=PathJoinSubstitution([FindPackageShare('aurora_bringup'), 'config', 'fast_lio', 'avia_mid70.yaml'])),

        DeclareLaunchArgument('zed_model', default_value=TextSubstitution(text='zed2i')),
        DeclareLaunchArgument('zed_resolution', default_value=TextSubstitution(text='HD720')),
        DeclareLaunchArgument('zed_fps', default_value=TextSubstitution(text='15')),
        DeclareLaunchArgument('zed_depth_mode', default_value=TextSubstitution(text='NEURAL')),

        DeclareLaunchArgument('lio_ref_frame', default_value=TextSubstitution(text='camera_init'), description='Reference frame published by FAST-LIO2'),
        DeclareLaunchArgument('publish_odom_identity', default_value='true', description='Publish identity TF odom->lio_ref_frame'),
        DeclareLaunchArgument('publish_map_identity', default_value='false', description='Publish identity TF map->odom'),

        # base_link -> livox_frame (default: co-located, no rotation)
        DeclareLaunchArgument('bl2livox_x', default_value='0.0'),
        DeclareLaunchArgument('bl2livox_y', default_value='0.0'),
        DeclareLaunchArgument('bl2livox_z', default_value='0.0'),
        DeclareLaunchArgument('bl2livox_roll', default_value='0.0'),
        DeclareLaunchArgument('bl2livox_pitch', default_value='0.0'),
        DeclareLaunchArgument('bl2livox_yaw', default_value='0.0'),

        # base_link -> zed_camera_center (default: simple forward mount)
        DeclareLaunchArgument('bl2zed_x', default_value='0.05'),
        DeclareLaunchArgument('bl2zed_y', default_value='0.0'),
        DeclareLaunchArgument('bl2zed_z', default_value='0.10'),
        DeclareLaunchArgument('bl2zed_roll', default_value='0.0'),
        DeclareLaunchArgument('bl2zed_pitch', default_value='0.0'),
        DeclareLaunchArgument('bl2zed_yaw', default_value='0.0'),

        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([FindPackageShare('aurora_bringup'), 'rviz', 'aurora_fastlio.rviz']))
    ]

    # ----------------------- MAVROS Node -----------------------
    mavros_nodes = []
    if True:
        mavros_share = get_package_share_directory('mavros')
        apm_config = os.path.join(mavros_share, 'launch', 'apm_config.yaml')
        apm_plugins = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')

        mavros_nodes = [
            Node(
                package='mavros',
                executable='mavros_node',
                namespace=namespace,
                output='screen',
                parameters=[
                    {'fcu_url': fcu_url, 'gcs_url': gcs_url},
                    apm_config, apm_plugins
                ],
                condition=IfCondition(use_mavros)
            )
        ]

    # ----------------------- Livox Driver -----------------------
    livox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('livox_ros2_driver'), 'launch', 'livox_lidar_launch.py'])
        ),
        launch_arguments={'config_path': livox_config}.items(),
        condition=IfCondition(use_livox)
    )

    # ----------------------- ZED Wrapper -----------------------
    zed_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])
        ),
        launch_arguments={
            'camera_model': zed_model,
            'camera_fps': zed_fps,
            'resolution': zed_resolution,
            'depth_mode': zed_depth_mode
        }.items(),
        condition=IfCondition(use_zed)
    )

    # ----------------------- FAST-LIO2 -----------------------
    # Uses installed fast_lio package's mapping.launch.py
    fastlio_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])
        ),
        launch_arguments={
            'config_file': fastlio_config
        }.items(),
        condition=IfCondition(use_fastlio)
    )

    # ----------------------- Static TF Publishers -----------------------
    # 1) Identity odom -> lio_ref_frame (to unify RViz fixed frame 'odom' with LIO's 'camera_init')
    tf_odom_to_lio = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_lio_ref',
        arguments=['0','0','0','0','0','0','odom', lio_ref_frame],
        condition=IfCondition(publish_odom_identity)
    )

    # 2) Optional identity map -> odom
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['0','0','0','0','0','0','map','odom'],
        condition=IfCondition(publish_map_identity)
    )

    # 3) base_link -> livox_frame
    tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox',
        arguments=[bl2livox_x, bl2livox_y, bl2livox_z, bl2livox_roll, bl2livox_pitch, bl2livox_yaw, 'base_link', 'livox_frame'],
        condition=IfCondition(use_livox)
    )

    # 4) base_link -> zed_camera_center
    tf_base_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed',
        arguments=[bl2zed_x, bl2zed_y, bl2zed_z, bl2zed_roll, bl2zed_pitch, bl2zed_yaw, 'base_link', 'zed_camera_center'],
        condition=IfCondition(use_zed)
    )

    # ----------------------- RViz2 -----------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    # ----------------------- Rosbag2 (optional) -----------------------
    bag_all = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        shell=False,
        condition=IfCondition(record_all)
    )

    # Optional grouping under namespace
    group = GroupAction([
        PushRosNamespace(namespace),
        livox_include,
        zed_include,
        fastlio_include,
        tf_odom_to_lio,
        tf_map_to_odom,
        tf_base_to_livox,
        tf_base_to_zed,
        rviz,
        bag_all,
    ] + mavros_nodes)

    return LaunchDescription(declares + [group])
