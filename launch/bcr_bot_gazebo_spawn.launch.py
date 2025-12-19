# #!/usr/bin/env python3

# from os.path import join
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
# from launch.substitutions import LaunchConfiguration, Command, PythonExpression
# from launch_ros.actions import Node
# from nav2_common.launch import RewrittenYaml
# from launch_ros.descriptions import ParameterFile


# def generate_robot_group1(robot_name, x, y, yaw):
#     """Creates a group of nodes for one robot instance"""
#     bcr_bot_path = get_package_share_directory('bcr_bot')
#     xacro_path1 = join(bcr_bot_path, 'urdf', 'bcr_bot_description', 'bcr_bot.xacro')
#     twist_mux_params = os.path.join(bcr_bot_path, 'config', 'twist_mux.yaml')
#    # rsp_params = os.path.join(bcr_bot_path, 'config', 'rsp.yaml')
#     # robot_namespace = f"{robot_name}"
#     # Robot State Publisher
#     rsp = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         namespace=robot_name,
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'use_sim_time': True,
#             'publish_frequency': 30.0,
#             'robot_description': Command([
#                 'xacro ', xacro_path1,
#                 ' camera_enabled:=true',
#                 ' stereo_camera_enabled:=false',
#                 ' two_d_lidar_enabled:=true',
#                 ' sim_gazebo:=true',
#                 ' odometry_source:=world',
#                 ''
#                 ' robot_namespace:=', robot_name,

#             ])
#         }],
#         remappings=[
#             ('/joint_states', f'/{robot_name}/joint_states'),
#         ]
#     )

#     # Gazebo spawn
#     spawn = TimerAction(
#         period = 3.0,
#         actions = [Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         namespace=robot_name,
#         output='screen',
#         arguments=[
#             '-topic', f"/{robot_name}/robot_description",
#             '-entity', f'{robot_name}_robot',
#             '-z', "0.28",
#             '-x', str(x),
#             '-y', str(y),
#             '-Y', str(yaw),
#         ]
#     )])

#     twist_mux_param_subs = {
#         #'output_topic': f'/{robot_name}/cmd_vel',
#         'topics/navigation/topic': f'/{robot_name}/cmd_vel_collision',
#         'topics/joystick/topic': f'/{robot_name}/cmd_vel_teleop',
#         # add other input topics if needed
#     }

#     configured_params = ParameterFile(
#         RewrittenYaml(
#             source_file=twist_mux_params,
#             root_key= robot_name,   # must match top-level key in twist_mux.yaml
#             param_rewrites=twist_mux_param_subs,
#             convert_types=True
#         ),
#         allow_substs=True
#     )

#     twist_mux = Node(
#         package="twist_mux",
#         executable="twist_mux",
#         namespace=robot_name,
#         parameters=[configured_params, {'use_sim_time': True}],
#         remappings=[('cmd_vel_out', 'cmd_vel')]
#     )



#     return GroupAction([rsp, spawn, twist_mux])


# def generate_launch_description():
#     return LaunchDescription([
#         # Spawn 3 robots at different positions
#         generate_robot_group1("robot1", -7.0, 4.0, 0.0),
#         generate_robot_group1("robot2", -7.0, -4.5, 0.0),
#         #generate_robot_group("robot3", -7.0, -0.75, 0.0),

#         # generate_robot_group1("robot1", 0.0, 0.0, 0.0),
#         # generate_robot_group1("robot2", 0.0, -2.0, 0.0),
#     ])
















# #!/usr/bin/env python3

# from os.path import join
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node
# from nav2_common.launch import RewrittenYaml
# from launch_ros.descriptions import ParameterFile


# def generate_mir_robot_group(robot_name, x, y, yaw):
#     """Creates a group of nodes for one mir robot instance"""
#     bcr_bot_path = get_package_share_directory('bcr_bot')
#     # Option A - top-level mir xacro (as you selected)
#     xacro_path = join(bcr_bot_path, 'urdf', 'mir_description', 'mir.urdf.xacro')

#     twist_mux_params = os.path.join(bcr_bot_path, 'config', 'twist_mux.yaml')

#     # Robot State Publisher (provides robot_description topic in the robot namespace)
#     rsp = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         namespace=robot_name,
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'use_sim_time': True,
#             'publish_frequency': 30.0,
#             # Render the xacro passing tf_prefix so frames become robot_name/...
#             'robot_description': Command([
#                 'xacro ',
#                 xacro_path,
#                 ' ',
#                 'tf_prefix:=', robot_name ,
#                 ' ',
#                 'prefix:=', robot_name ,
#             ])
#         }],
#         remappings=[
#             # make sure joint_states get namespaced
#             ('/joint_states', f'/{robot_name}/joint_states'),
#         ]
#     )

#     # Gazebo spawn (wait a bit so robot_description is published)
#     spawn = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package='gazebo_ros',
#                 executable='spawn_entity.py',
#                 namespace=robot_name,
#                 output='screen',
#                 arguments=[
#                     '-topic', f"/{robot_name}/robot_description",
#                     '-entity', f'{robot_name}',
#                     '-z', "0.28",
#                     '-x', str(x),
#                     '-y', str(y),
#                     '-Y', str(yaw),
#                 ]
#             )
#         ]
#     )

#     # Rewrites for twist_mux: adapt the yaml's root to this robot namespace
#     twist_mux_param_subs = {
#         # example replacements — adjust keys to match your twist_mux.yaml structure
#         'topics/navigation/topic': f'/{robot_name}/cmd_vel_collision',
#         'topics/joystick/topic': f'/{robot_name}/cmd_vel_teleop',
#     }

#     configured_params = ParameterFile(
#         RewrittenYaml(
#             source_file=twist_mux_params,
#             root_key=robot_name,   # must match top-level key in twist_mux.yaml
#             param_rewrites=twist_mux_param_subs,
#             convert_types=True
#         ),
#         allow_substs=True
#     )

#     twist_mux = Node(
#         package="twist_mux",
#         executable="twist_mux",
#         namespace=robot_name,
#         parameters=[configured_params, {'use_sim_time': True}],
#         remappings=[('cmd_vel_out', 'cmd_vel')]
#     )

#     return GroupAction([rsp, spawn, twist_mux])


# def generate_launch_description():
#     ld = LaunchDescription()

#     # Spawn mirbot1 at the desired pose
#     ld.add_action(generate_mir_robot_group("mirbot1", -7.0, 4.0, 0.0))

#     # Example: to spawn additional robots, uncomment/add:
#     # ld.add_action(generate_mir_robot_group("mirbot2", -7.0, -4.5, 0.0))
#     # ld.add_action(generate_mir_robot_group("mirbot3", -7.0, -0.75, 0.0))

#     return ld









#!/usr/bin/env python3

from os.path import join
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile

# ============================================================
# -------------------- BCR ROBOT GROUP ------------------------
# ============================================================

def generate_bcr_robot_group(robot_name1, x, y, yaw):
    bcr_bot_path = get_package_share_directory('bcr_bot')
    xacro_path = join(bcr_bot_path, 'urdf', 'bcr_bot_description', 'bcr_bot.xacro')
    twist_mux_params = os.path.join(bcr_bot_path, 'config', 'twist_mux.yaml')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name1,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_frequency': 30.0,
            'robot_description': Command([
                'xacro ', xacro_path,
                ' sim_gazebo:=true',
                ' two_d_lidar_enabled:=true',
                ' camera_enabled:=true',
                ' stereo_camera_enabled:=false',
                ' prefix:=', robot_name1,
                ' tf_prefix:=', robot_name1,
                ' robot_namespace:=', robot_name1,
            ])
        }],
        remappings=[
            ('/joint_states', f'/{robot_name1}/joint_states'),
        ]
    )

    # Spawn in Gazebo
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace=robot_name1,
                output='screen',
                arguments=[
                    '-topic', f'/{robot_name1}/robot_description',
                    '-entity', robot_name1,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.28',
                    '-Y', str(yaw),
                ]
            )
        ]
    )

    # Twist mux
    twist_mux_param_subs = {
        'topics/navigation/topic': f'/{robot_name1}/cmd_vel_collision',
        'topics/joystick/topic': f'/{robot_name1}/cmd_vel_teleop',
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=twist_mux_params,
            root_key=robot_name1,
            param_rewrites=twist_mux_param_subs,
            convert_types=True
        ),
        allow_substs=True
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=robot_name1,
        parameters=[configured_params, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')]
    )

    return GroupAction([rsp, spawn, twist_mux])


# ============================================================
# -------------------- MIR ROBOT GROUP ------------------------
# ============================================================

def generate_mir_robot_group(robot_name2, x, y, yaw):
    bcr_bot_path = get_package_share_directory('bcr_bot')
    xacro_path = join(bcr_bot_path, 'urdf', 'mir_description', 'mir.urdf.xacro')
    twist_mux_params = os.path.join(bcr_bot_path, 'config', 'twist_mux.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name2,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_frequency': 30.0,
            'robot_description': Command([
                'xacro ', xacro_path,
                ' prefix:=', robot_name2,
                ' tf_prefix:=', robot_name2,
            ])
        }],
        remappings=[
            ('/joint_states', f'/{robot_name2}/joint_states'),
        ]
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace=robot_name2,
                output='screen',
                arguments=[
                    '-topic', f'/{robot_name2}/robot_description',
                    '-entity', robot_name2,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.28',
                    '-Y', str(yaw),
                ]
            )
        ]
    )

    twist_mux_param_subs = {
        'topics/navigation/topic': f'/{robot_name2}/cmd_vel_collision',
        'topics/joystick/topic': f'/{robot_name2}/cmd_vel_teleop',
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=twist_mux_params,
            root_key=robot_name2,
            param_rewrites=twist_mux_param_subs,
            convert_types=True
        ),
        allow_substs=True
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=robot_name2,
        parameters=[configured_params, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')]
    )

    return GroupAction([rsp, spawn, twist_mux])


# ============================================================
# -------------------- MAIN LAUNCH ----------------------------
# ============================================================

def generate_launch_description():
    ld = LaunchDescription()

    # ---- BCR Robots ----
    ld.add_action(generate_bcr_robot_group('robot1', -7.0, 4.0, 0.0))
    #ld.add_action(generate_bcr_robot_group('robot2', -7.0, -4.5, 0.0))

    # ---- MiR Robots ----
    ld.add_action(generate_mir_robot_group('robot2', 5.0, -4.0, 3.14))

    return ld
