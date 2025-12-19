#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('bcr_bot')

    # ---------------------------
    # Launch Configurations
    # ---------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Robots to spawn collision monitor for


    lifecycle_nodes = ['collision_monitor']

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # ---------------------------
    # Launch Arguments
    # ---------------------------
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'collision_monitor.yaml'),
        description='Full path to collision monitor params file')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Restart nodes if they crash')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Auto-start lifecycle nodes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Logging level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # ---------------------------
    # Launch Description Init
    # ---------------------------
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)

    robots = [
        {'name': 'robot1'},
        #{'name': 'robot2'},
        # {'name': 'robot3'},
        {'name': 'robot2'},
    ]
    
    robot_nodes = []
    # ---------------------------
    # Per-Robot Setup Loop
    # ---------------------------
    for robot in robots:
        ns = robot['name']

        # Per-robot parameter overrides
        robot_param_subs = {
            'use_sim_time': use_sim_time,
            'base_frame_id': f'{ns}/base_footprint',
            'odom_frame_id': f'{ns}/odom',
            'topic': f'/{ns}/scan',
            'cmd_vel_in_topic': f'/{ns}/cmd_vel_smoothed',
            'cmd_vel_out_topic': f'/{ns}/cmd_vel_collision',
            'PolygonStop/polygon_pub_topic': f'{ns}/polygon_stop',
            'PolygonSlow/polygon_pub_topic': f'{ns}/polygon_slowdown',
        }

        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key=ns,
                param_rewrites=robot_param_subs,
                convert_types=True),
            allow_substs=True)

        # ---------------------------
        # Collision Monitor Node
        # ---------------------------
        collision_monitor_node = Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            namespace=ns,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            #remappings=[('polygon_stop', f'{ns}/polygon_stop' ),('polygon_slowdown', f'{ns}/polygon_slowdown' )],
        )
        #ld.add_action(collision_monitor_node)
        robot_nodes.append(collision_monitor_node)

        # ---------------------------
        # Custom Polygon Monitor Node
        # ---------------------------
        polygon_monitor_node = Node(
            package='bcr_bot',
            executable='collision_detector.py',
            name='collision_detector',
            namespace=ns,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('scan', f'/{ns}/scan' ),('polygon_slowdown', f'/{ns}/polygon_slowdown' )],
        )
        #ld.add_action(polygon_monitor_node)
        robot_nodes.append(polygon_monitor_node)

        # ---------------------------
        # Lifecycle Manager
        # ---------------------------
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes
            }],
            arguments=['--ros-args', '--log-level', log_level]
        )
        #ld.add_action(lifecycle_manager_node)
        robot_nodes.append(lifecycle_manager_node)

    for node in robot_nodes:
        ld.add_action(node)

    return ld

