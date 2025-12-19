#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('bcr_bot')
    params_file = os.path.join(bringup_dir, 'config', 'dual_laser_merger.yaml')

    robots = [
        {'name': 'robot2'},
    ]

    ld = LaunchDescription()

    for robot in robots:
        ns = robot['name']

        # rewrite parameters per robot
        robot_param_subs = {
            'target_frame': f'{ns}/base_link',
        }

        robot_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key=ns,
                param_rewrites=robot_param_subs,
                convert_types=True
            ),
            allow_substs=True
        )

        # component container (THIS IS REQUIRED)
        merger_container = ComposableNodeContainer(
            name='dual_laser_merger_container',
            namespace=ns,
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='dual_laser_merger',
                    plugin='merger_node::MergerNode',
                    name='laser_merger',
                    parameters=[params_file],
                    remappings=[
                        ('f_scan', f'/{ns}/f_scan'),
                        ('b_scan', f'/{ns}/b_scan'),
                        ('scan',   f'/{ns}/scan'),
                    ],
                )
            ]
        )

        ld.add_action(merger_container)

    return ld
