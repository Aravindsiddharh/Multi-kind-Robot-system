
# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch_ros.actions import LoadComposableNodes
# from launch_ros.actions import Node
# from launch_ros.descriptions import ComposableNode, ParameterFile
# from nav2_common.launch import RewrittenYaml


# def generate_launch_description():
#     # Get the launch directory
#     bringup_dir = get_package_share_directory('bcr_bot')

#     namespace = LaunchConfiguration('namespace')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     autostart = LaunchConfiguration('autostart')
#     params_file = LaunchConfiguration('params_file')
#     use_composition = LaunchConfiguration('use_composition')
#     container_name = LaunchConfiguration('container_name')
#     container_name_full = (namespace, '/', container_name)
#     use_respawn = LaunchConfiguration('use_respawn')
#     log_level = LaunchConfiguration('log_level')

#     lifecycle_nodes = ['controller_server',
#                        'smoother_server',
#                        'planner_server',
#                        'behavior_server',
#                        'bt_navigator',
#                        'waypoint_follower',
#                        'velocity_smoother',
#                        'route_server']
#     # lifecycle_nodes_1 = ['amcl',
#     #                      'map_server']
#     # Map fully qualified names to relative ones so the node's namespace can be prepended.
#     # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
#     # https://github.com/ros/geometry2/issues/32
#     # https://github.com/ros/robot_state_publisher/pull/30
#     # TODO(orduno) Substitute with `PushNodeRemapping`
#     #              https://github.com/ros2/launch_ros/issues/56
#     remappings = [('/tf', 'tf'),
#                   ('/tf_static', 'tf_static')]

#     # Create our own temporary YAML files that include substitutions
#     param_substitutions = {
#         'use_sim_time': use_sim_time,
#         'autostart': autostart}

#     configured_params = ParameterFile(
#         RewrittenYaml(
#             source_file=params_file,
#             root_key=namespace,
#             param_rewrites=param_substitutions,
#             convert_types=True),
#         allow_substs=True)

#     stdout_linebuf_envvar = SetEnvironmentVariable(
#         'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

#     declare_namespace_cmd = DeclareLaunchArgument(
#         'namespace',
#         default_value='',
#         description='Top-level namespace')

#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation (Gazebo) clock if true')

#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
#         description='Full path to the ROS2 parameters file to use for all launched nodes')

#     declare_autostart_cmd = DeclareLaunchArgument(
#         'autostart', default_value='true',
#         description='Automatically startup the nav2 stack')

#     declare_use_composition_cmd = DeclareLaunchArgument(
#         'use_composition', default_value='False',
#         description='Use composed bringup if True')

#     declare_container_name_cmd = DeclareLaunchArgument(
#         'container_name', default_value='nav2_container',
#         description='the name of conatiner that nodes will load in if use composition')

#     declare_use_respawn_cmd = DeclareLaunchArgument(
#         'use_respawn', default_value='False',
#         description='Whether to respawn if a node crashes. Applied when composition is disabled.')

#     declare_log_level_cmd = DeclareLaunchArgument(
#         'log_level', default_value='info',
#         description='log level')

#     load_nodes = GroupAction(
#         condition=IfCondition(PythonExpression(['not ', use_composition])),
#         actions=[
            # Node(
            #     package='nav2_controller',
            #     executable='controller_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params ],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings + [('cmd_vel', 'siddhu/cmd_vel')]),  # cmd_vel_nav #diff_cont/cmd_vel_unstamped
            # Node(
            #     package='nav2_smoother',
            #     executable='smoother_server',
            #     name='smoother_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings),
            # Node(
            #     package='nav2_planner',
            #     executable='planner_server',
            #     name='planner_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params, 
            #                 {'automatically_declare_parameters_from_overrides': True}],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings             #+[('plan', 'path'),]
            #     ),
            # Node(
            #     package='nav2_behaviors',
            #     executable='behavior_server',
            #     name='behavior_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings),
            # Node(
            #     package='nav2_bt_navigator',
            #     executable='bt_navigator',
            #     name='bt_navigator',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings),
            # Node(
            #     package='nav2_waypoint_follower',
            #     executable='waypoint_follower',
            #     name='waypoint_follower',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings),
            # Node(
            #     package='nav2_velocity_smoother',
            #     executable='velocity_smoother',
            #     name='velocity_smoother',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings +
            #             [('cmd_vel', 'siddhu/cmd_vel'), 
            #             ('cmd_vel_smoothed', 'mallumatta_cmd_vel')]), # if you want to enable collision_monitor comment this line else change the output velocity in gazebo launch file
            # Node(
            #     package='nav2_lifecycle_manager',
            #     executable='lifecycle_manager',
            #     name='lifecycle_manager_navigation',
            #     output='screen',
            #     arguments=['--ros-args', '--log-level', log_level],
            #     parameters=[{'use_sim_time': use_sim_time},
            #                 {'autostart': autostart},
            #                 {'node_names': lifecycle_nodes}]),
            # Node(
            #     package='turtlebot3_waypoint',
            #     executable='start_stop',
            #     name='start_stop',
            #     output='screen',
            # ),
            # Node(
            #     package='turtlebot3_waypoint',
            #     executable='planner_switch_node',
            #     name='planner_switch_node',
            #     output='screen',
            # ),
            #  Node(
            #     package='nav2_route',
            #     executable='route_server',
            #     name='route_server',
            #     output='screen',
            #     #respawn=use_respawn,
            #     #respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings   + [('plan', 'path')]
            # ),
          
#         ]
#     )

#     load_composable_nodes = LoadComposableNodes(
#         condition=IfCondition(use_composition),
#         target_container=container_name_full,
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='nav2_controller',
#                 plugin='nav2_controller::ControllerServer',
#                 name='controller_server',
#                 parameters=[configured_params],
#                 remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
#             ComposableNode(
#                 package='nav2_smoother',
#                 plugin='nav2_smoother::SmootherServer',
#                 name='smoother_server',
#                 parameters=[configured_params],
#                 remappings=remappings),
#             ComposableNode(
#                 package='nav2_planner',
#                 plugin='nav2_planner::PlannerServer',
#                 name='planner_server',
#                 parameters=[configured_params],
#                 remappings=remappings),
#             ComposableNode(
#                 package='nav2_behaviors',
#                 plugin='behavior_server::BehaviorServer',
#                 name='behavior_server',
#                 parameters=[configured_params],
#                 remappings=remappings),
#             ComposableNode(
#                 package='nav2_bt_navigator',
#                 plugin='nav2_bt_navigator::BtNavigator',
#                 name='bt_navigator',
#                 parameters=[configured_params],
#                 remappings=remappings),
#             ComposableNode(
#                 package='nav2_waypoint_follower',
#                 plugin='nav2_waypoint_follower::WaypointFollower',
#                 name='waypoint_follower',
#                 parameters=[configured_params],
#                 remappings=remappings),
#             ComposableNode(
#                 package='nav2_velocity_smoother',
#                 plugin='nav2_velocity_smoother::VelocitySmoother',
#                 name='velocity_smoother',
#                 parameters=[configured_params],
#                 remappings=remappings +
#                            [('cmd_vel', 'cmd_vel_nav'), 
#                             ('cmd_vel_smoothed', 'diff_cont/cmd_vel_unstamped')]),# if you want to enable collision_monitor comment this line else change the output velocity in gazebo launch file
#             ComposableNode(
#                 package='nav2_lifecycle_manager',
#                 plugin='nav2_lifecycle_manager::LifecycleManager',
#                 name='lifecycle_manager_navigation',
#                 parameters=[{'use_sim_time': use_sim_time,
#                              'autostart': autostart,
#                              'node_names': lifecycle_nodes}]),
#             ComposableNode(
#                 package='nav2_route',
#                 plugin='nav2_route::RouteServer',
#                 name='route_server',
#                 parameters=[configured_params],
#                 remappings=remappings),

#         ],
#     )

#     # Create the launch description and populate
#     ld = LaunchDescription()

#     # Set environment variables
#     ld.add_action(stdout_linebuf_envvar)

#     # Declare the launch options
#     ld.add_action(declare_namespace_cmd)
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(declare_autostart_cmd)
#     ld.add_action(declare_use_composition_cmd)
#     ld.add_action(declare_container_name_cmd)
#     ld.add_action(declare_use_respawn_cmd)
#     ld.add_action(declare_log_level_cmd)
#     # Add the actions to launch all of the navigation nodes
#     ld.add_action(load_nodes)
#     ld.add_action(load_composable_nodes)

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

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    #declare_map_yaml_cmd = DeclareLaunchArgument('map', description='Full path to map yaml file to load')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params1.yaml'),
        description='ROS2 params file for Nav2'
    )
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup nav2')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn if node crashes')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', description='Log level')

    # remappings_global = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # # Global map server
    # global_param_subs = {
    #     'use_sim_time': use_sim_time,
    #     'yaml_filename': map_yaml_file
    # }
    # configured_map_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,
    #         root_key='',
    #         param_rewrites=global_param_subs,
    #         convert_types=True
    #     ),
    #     allow_substs=True
    # )
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     respawn=use_respawn,
    #     respawn_delay=2.0,
    #     parameters=[configured_map_params],
    #     remappings=remappings_global
    # )
    # global_lifecycle_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_map',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': autostart,
    #         'node_names': ['map_server']
    #     }]
    # )

    # Robots
    robots = [
        {'name': 'robot1',},
        #{'name': 'robot2', },
        #{'name' : 'robot3', 'x': -7.0, 'y': -0.75,'yaw': 0.0},
        {'name': 'robot2', }
    ]
    robot_nodes = []

    for robot in robots:
        ns = robot['name']

        # Per-robot Nav2 parameter rewriting
        # robot_param_subs = {
        #     'use_sim_time': use_sim_time,
        #     'yaml_filename': map_yaml_file,
        #     'global_frame': f'{ns}/odom',
        #     'base_frame': f'{ns}/base_link',
        #     'robot_base_frame': f'{ns}/base_link',  
        #     'global_frame': 'map',
        #     'map_topic': '/map'
        # }
        robot_param_subs = {
        'use_sim_time': use_sim_time,
        'global_frame': 'map',                     # shared
        'route_frame': 'map',            # default
        'base_frame': f'{ns}/base_link',       # default (for planner etc.)
        'map_topic': '/map',
        'odom_topic': f'/{ns}/odom',
        'robot_base_frame': f'{ns}/base_link',
        'topic': f'/{ns}/scan',          # default (for local costmap etc.)
        'scan_frame': f'{ns}/two_d_lidar',

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

      

        nav2_nodes = ['controller_server', 'smoother_server', 'planner_server', 'bt_navigator', 
                    'waypoint_follower', 'velocity_smoother', 'behavior_server', 'route_server']

    
        

        controller_server_node = Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            )
        robot_nodes.append(controller_server_node)
        
        smoother_node = Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings_global
            )
        robot_nodes.append(smoother_node)
        
        planner_server_node= Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings = remappings_global + [('plan', f'{ns}/plan')],
            )
        robot_nodes.append(planner_server_node)

        behavior_server = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings
            )
        robot_nodes.append(behavior_server)
        
        bt_navigator_node = TimerAction(
            period = 5.0,
            actions = [Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings= [('/goal_pose', f'/{ns}/goal_pose')]
            )])
        robot_nodes.append(bt_navigator_node)

        waypoint_follower = Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                namespace=ns,
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
            )
        robot_nodes.append(waypoint_follower)
        
        velocity_smoother_node = Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                namespace=ns,    
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav'),] 
                            #('cmd_vel_smoothed', f'{ns}/diff_cont/cmd_vel_unstamped')]
            ) # if you want to enable collision_monitor comment this line else change the output velocity in gazebo launch file
        route_server = Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                namespace=ns,
                output='screen',
                respawn=use_respawn,

                respawn_delay=2.0,
                parameters=[robot_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('plan', f'{ns}/path')]
        )
        robot_nodes.append(route_server)
        
        robot_nodes.append(velocity_smoother_node)
        # Lifecycle manager
        lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_navigation_{ns}',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': nav2_nodes
            }]
        )
        robot_nodes.append(lifecycle_node)

        start_stop_node = Node(
            package='bcr_bot',
            executable='start_stop.py',
            name='start_stop',
            namespace=ns,
            output='screen',
            remappings=[('nav_control', f'/{ns}/nav_control'),
                        ('goal_pose', f'/{ns}/goal_pose')]
        )
        robot_nodes.append(start_stop_node)

        goal_cancel_node = Node(
            package='bcr_bot',
            executable='goal_cancel_node.py',
            name='goal_cancel',
            namespace=ns,
            output='screen',
            remappings=[('nav_cancel', f'/{ns}/nav_cancel')]
        )
        robot_nodes.append(goal_cancel_node)        

        controller_switch_node = Node(
            package='bcr_bot',
            executable='controller_switch_node.py',
            name='controller_switch',
            namespace=ns,
            output='screen',
            remappings=[('change_controller', f'/{ns}/change_controller'),
                        ('controller_selector', f'/{ns}/controller_selector')]
        )
        robot_nodes.append(controller_switch_node)
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    #ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # # Add global map server
    # ld.add_action(map_server_node)
    # ld.add_action(global_lifecycle_node)

    # Add all per-robot nodes
    for node in robot_nodes:
        ld.add_action(node)

    return ld
