
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import (
    Node,
    PushROSNamespace,
    LoadComposableNodes,
)

from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('bcr_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    autostart = LaunchConfiguration('autostart')

    robots = [
        'robot1',
        # 'robot2',
    ]

    lifecycle_nodes = ['collision_monitor']

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True'
    ))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir,
            'config',
            'collision_monitor.yaml'
        )
    ))

    ld.add_action(DeclareLaunchArgument(
        'use_composition',
        default_value='False'
    ))

    ld.add_action(DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container'
    ))

    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='True'
    ))

    for robot in robots:

        ns = robot

        # Robot-specific parameters
        robot_param_subs = {
            'use_sim_time': use_sim_time,
            'base_frame_id': f'{ns}/base_link',
            'odom_frame_id': f'{ns}/odom',
            'topic': f'/{ns}/scan',
            'cmd_vel_in_topic': f'/{ns}/cmd_vel_smoothed',
            'cmd_vel_out_topic': f'/{ns}/cmd_vel',
            'PolygonStop/polygon_pub_topic': f'{ns}/polygon_stop',
            'PolygonSlow/polygon_pub_topic': f'{ns}/polygon_slowdown',
        }

        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key=ns,
                param_rewrites=robot_param_subs,
                convert_types=True,
            ),
            allow_substs=True,
        )

        # -------------------------------
        # Standalone mode
        # -------------------------------
        standalone_nodes = GroupAction(
            condition=UnlessCondition(use_composition),
            actions=[
                PushROSNamespace(ns),

                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name='collision_monitor',
                    output='screen',
                    parameters=[configured_params],
                    remappings=remappings,
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_collision_monitor',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes},
                    ],
                    remappings=remappings,
                ),
            ],
        )

        ld.add_action(standalone_nodes)

        # -------------------------------
        # Composition mode
        # -------------------------------
        composable_nodes = GroupAction(
            condition=IfCondition(use_composition),
            actions=[
                LoadComposableNodes(
                    target_container=[
                        '/', ns, '/', container_name
                    ],
                    composable_node_descriptions=[
                        ComposableNode(
                            package='nav2_collision_monitor',
                            plugin=(
                                'nav2_collision_monitor'
                                '::CollisionMonitor'
                            ),
                            namespace=ns,
                            name='collision_monitor',
                            parameters=[configured_params],
                            remappings=remappings,
                        ),

                        ComposableNode(
                            package='nav2_lifecycle_manager',
                            plugin=(
                                'nav2_lifecycle_manager::LifecycleManager'
                            ),
                            namespace=ns,
                            name='lifecycle_manager_collision_monitor',
                            parameters=[
                                {'use_sim_time': use_sim_time},
                                {'autostart': False},
                                {'node_names': lifecycle_nodes},
                            ],
                            remappings=remappings,
                        ),
                    ],
                ),
            ],
        )

        ld.add_action(composable_nodes)

    return ld
