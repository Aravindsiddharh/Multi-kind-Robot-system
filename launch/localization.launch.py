# #!/usr/bin/env python3

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.descriptions import ParameterFile
# from nav2_common.launch import RewrittenYaml

# # def generate_robot_initial_pose_node(robot_name, x, y, yaw):
# #     """Return a Node action that publishes the robot's initial pose"""
# #     return TimerAction(
# #     period=2.0, 
# #     actions =[ Node(
# #         package="bcr_bot",
# #         executable="initial_pose_publisher.py",
# #         name=f"init_pose_{robot_name}",
# #         arguments=[robot_name, str(x), str(y), str(yaw)],
# #         output="screen",
# #     )])

# def generate_launch_description():
#     bringup_dir = get_package_share_directory('bcr_bot')

#     # Launch arguments
#     namespace = LaunchConfiguration('namespace')
#     map_yaml_file = LaunchConfiguration('map')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     autostart = LaunchConfiguration('autostart')
#     params_file = LaunchConfiguration('params_file')
#     use_respawn = LaunchConfiguration('use_respawn')
#     log_level = LaunchConfiguration('log_level')
#     remappings_global = [('/tf', 'tf'),
#                     ('/tf_static', 'tf_static')]
#     # Environment variable for logging
#     stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

#     # Declare launch arguments
#     declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
#     declare_map_yaml_cmd = DeclareLaunchArgument('map', description='Full path to map yaml file to load')
#     declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true')
#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file', 
#         default_value=os.path.join(bringup_dir, 'config', 'amcl_params.yaml'), 
#         description='ROS2 params file'
#     )
#     declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup nav2')
#     declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn if node crashes')
#     declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', description='Log level')

#     # Global map server (shared)
#     global_param_subs = {
#         'use_sim_time': use_sim_time,
#         'yaml_filename': map_yaml_file
#     }
#     configured_map_params = ParameterFile(
#         RewrittenYaml(
#             source_file=params_file,
#             root_key='',
#             param_rewrites=global_param_subs,
#             convert_types=True
#         ),
#         allow_substs=True
#     )
#     map_server_node = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         name='map_server',
#         output='screen',
#         respawn=use_respawn,
#         respawn_delay=2.0,
#         parameters=[configured_map_params],
#         arguments=['--ros-args', '--log-level', log_level],
#         remappings=remappings_global
#     )
#     global_lifecycle_node = Node(
#                 package='nav2_lifecycle_manager',
#                 executable='lifecycle_manager',
#                 name='lifecycle_manager_localization',
#                 output='screen',
#                 arguments=['--ros-args', '--log-level', log_level],
#                 parameters=[{'use_sim_time': use_sim_time},
#                             {'autostart': autostart},
#                             {'node_names': ['map_server']}]
#     )
        
#     # Robots
#     # robots = [
#     #     {'name': 'robot1', 'x': 5.0, 'y': -4.0, 'yaw': 1.57},
#     #     {'name': 'robot2', 'x': -5.0, 'y': -4.0, 'yaw': 1.57}
#     # ]
#     robots = [
#         {'name': 'robot1',},
#         {'name': 'robot2',}
#     ]
#     robot_nodes = []
#     for robot in robots:
#         ns = robot['name']

#         # Per-robot AMCL parameter rewriting
#         robot_param_subs = {
#             'use_sim_time': use_sim_time,
#             'yaml_filename': map_yaml_file,
#             'odom_frame_id': f'{ns}/odom',
#             'base_frame_id': f'{ns}/base_link',
#             'global_frame_id': 'map',
#             'map_topic': 'map'


#         }
#         robot_params = ParameterFile(
#             RewrittenYaml(
#                 source_file=params_file,
#                 root_key=ns,
#                 param_rewrites=robot_param_subs,
#                 convert_types=True
#             ),
#             allow_substs=True
#         )

#         # Remappings for this robot
#         remappings = [
#             ('/scan', f'/{ns}/scan'),
#             # ('/tf', f'/{ns}/tf'),
#             # ('/tf_static', f'/{ns}/tf_static')
#         ]

#         # AMCL node (delayed start)
#         amcl_node = Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             namespace=ns,
#             output='screen',
#             respawn=use_respawn,
#             respawn_delay=2.0,
#             parameters=[robot_params],
#             arguments=['--ros-args', '--log-level', log_level],
#             remappings=remappings
#         )
#         robot_nodes.append(TimerAction(
#             period=3.0,
#             actions=[amcl_node]
#         ))

#         # Lifecycle manager
#         lifecycle_node = Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             namespace=ns,
#             output='screen',
#             parameters=[{
#                 'use_sim_time': use_sim_time,
#                 'autostart': autostart,
#                 'node_names': ['amcl']
#             }],
#             arguments=['--ros-args', '--log-level', log_level]
#         )
#         robot_nodes.append(lifecycle_node)

#         # Static transform from map -> odom
#         # robot_nodes.append(Node(
#         #     package='tf2_ros',
#         #     executable='static_transform_publisher',
#         #     name=f'static_tf_{ns}',
#         #     output='screen',
#         #     arguments=['0', '0', '0', '0', '0', '0', 'map', f'/{ns}/odom']
#         # ))

#         # Initial pose publisher
#         #robot_nodes.append(generate_robot_initial_pose_node(ns, robot['x'], robot['y'], robot['yaw']))

#     # Create launch description
#     ld = LaunchDescription()
#     ld.add_action(stdout_linebuf_envvar)
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(declare_map_yaml_cmd)
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(declare_autostart_cmd)
#     ld.add_action(declare_use_respawn_cmd)
#     ld.add_action(declare_log_level_cmd)

#     # Add global map server
#     ld.add_action(map_server_node)
#     ld.add_action(global_lifecycle_node)

#     # Add per-robot nodes
#     for node in robot_nodes:
#         ld.add_action(node)

#     return ld





















#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('bcr_bot')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    remappings_global = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', description='Full path to map yaml file to load')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(bringup_dir, 'config', 'amcl_params.yaml'), 
        description='ROS2 params file'
    )
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup nav2')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn if node crashes')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', description='Log level')

    # Global map server
    global_param_subs = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }
    configured_map_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=global_param_subs,
            convert_types=True
        ),
        allow_substs=True
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_map_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings_global
    )
    global_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server']}]
    )

    # Robots
    robots = [
        {'name': 'robot1', 'x': -7.0, 'y': 4.0, 'z': 0.0 ,'yaw': 0.0},
        # {'name': 'robot2', 'x': -7.0, 'y': -4.5, 'z': 0.0 ,'yaw': 0.0},
        {'name': 'robot2', 'x': 5.0, 'y': -4.0, 'z': 0.0 ,'yaw': 3.14}
 

    ]
    robot_nodes = []

    for robot in robots:
        ns = robot['name']

        # Per-robot AMCL parameter rewriting
        robot_param_subs = {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
            'odom_frame_id': f'{ns}/odom',
            'base_frame_id': f'{ns}/base_link',
            'global_frame_id': 'map',
            'map_topic': '/map',
            
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

        # Remappings for this robot
        remappings = [
            ('/scan', f'/{ns}/scan'),
            ('/map', '/map'),  # remap AMCL map topic to global map
        ]

        # AMCL node (delayed start)
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[robot_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        )
        robot_nodes.append(TimerAction(period=3.0, actions=[amcl_node]))

        # Lifecycle manager for AMCL
        lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_localization_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['amcl']
            }],
            arguments=['--ros-args', '--log-level', log_level]
        )
        robot_nodes.append(lifecycle_node)

        # Static transform: map -> odom
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{ns}',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'/{ns}/odom']
        )
        robot_nodes.append(static_tf_node)

        # Optional initial pose publisher (to set AMCL initial pose automatically)
        init_pose_node = Node(
            package='bcr_bot',
            executable='initial_pose_publisher.py',
            name=f'init_pose_{ns}',
            arguments=[ns, str(robot['x']), str(robot['y']), str(robot['z']), str(robot['yaw'])],
            output='screen'
        )
        robot_nodes.append(TimerAction(period=6.0, actions=[init_pose_node]))

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add global map server
    ld.add_action(map_server_node)
    ld.add_action(global_lifecycle_node)

    # Add per-robot nodes
    for node in robot_nodes:
        ld.add_action(node)

    return ld

