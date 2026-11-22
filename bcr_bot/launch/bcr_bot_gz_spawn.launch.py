#!/usr/bin/python3

# from os.path import join
# from xacro import parse, process_doc

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, Command

# from launch_ros.actions import Node

# from ament_index_python.packages import get_package_share_directory

# def get_xacro_to_doc(xacro_file_path, mappings):
#     doc = parse(open(xacro_file_path))
#     process_doc(doc, mappings=mappings)
#     return doc

# def generate_launch_description():
   
#     bcr_bot_path = get_package_share_directory("bcr_bot")
#     position_x = LaunchConfiguration("position_x")
#     position_y = LaunchConfiguration("position_y")
#     orientation_yaw = LaunchConfiguration("orientation_yaw")
#     camera_enabled = LaunchConfiguration("camera_enabled", default=True)
#     stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
#     two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
#     odometry_source = LaunchConfiguration("odometry_source")

#     # robot_description_content = get_xacro_to_doc(
#     #     join(bcr_bot_path, "urdf", "bcr_bot.xacro"),
#     #     {"sim_gz": "true",
#     #      "two_d_lidar_enabled": "true",
#     #      "conveyor_enabled": "false",
#     #      "camera_enabled": "true"
#     #     }
#     # ).toxml()

#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         parameters=[
#                     {'robot_description': Command( \
#                     ['xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
#                     ' camera_enabled:=', camera_enabled,
#                     ' stereo_camera_enabled:=', stereo_camera_enabled,
#                     ' two_d_lidar_enabled:=', two_d_lidar_enabled,
#                     ' odometry_source:=', odometry_source,
#                     ' sim_gz:=', "true"
#                     ])}],
#         remappings=[
#             ('/joint_states', 'bcr_bot/joint_states'),
#         ]
#     )

#     gz_spawn_entity = Node(
#         package="ros_gz_sim",
#         executable="create",
#         arguments=[
#             "-topic", "/robot_description",
#             "-name", "bcr_bot",
#             "-allow_renaming", "true",
#             "-z", "0.28",
#             "-x", position_x,
#             "-y", position_y,
#             "-Y", orientation_yaw
#         ]
#     )

#     gz_ros2_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         arguments=[
#             "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
#             "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
#             "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
#             "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
#             "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
#             "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
#             "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
#             "stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
#             "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
#             "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
#             "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
#             "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
#             "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
#             "/world/default/model/bcr_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
#         ],
#         remappings=[
#             ('/world/default/model/bcr_bot/joint_state', 'bcr_bot/joint_states'),
#             ('/odom', 'bcr_bot/odom'),
#             ('/scan', 'bcr_bot/scan'),
#             ('/kinect_camera', 'bcr_bot/kinect_camera'),
#             ('/stereo_camera/left/image_raw', 'bcr_bot/stereo_camera/left/image_raw'),
#             ('/stereo_camera/right/image_raw', 'bcr_bot/stereo_camera/right/image_raw'),
#             ('/imu', 'bcr_bot/imu'),
#             ('/cmd_vel', 'bcr_bot/cmd_vel'),
#             ('kinect_camera/camera_info', 'bcr_bot/kinect_camera/camera_info'),
#             ('stereo_camera/left/camera_info', 'bcr_bot/stereo_camera/left/camera_info'),
#             ('stereo_camera/right/camera_info', 'bcr_bot/stereo_camera/right/camera_info'),
#             ('/kinect_camera/points', 'bcr_bot/kinect_camera/points'),
#         ]
#     )

#     transform_publisher = Node(
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         arguments = ["--x", "0.0",
#                     "--y", "0.0",
#                     "--z", "0.0",
#                     "--yaw", "0.0",
#                     "--pitch", "0.0",
#                     "--roll", "0.0",
#                     "--frame-id", "kinect_camera",
#                     "--child-frame-id", "bcr_bot/base_footprint/kinect_camera"]
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
#         DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
#         DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
#         DeclareLaunchArgument("position_x", default_value="0.0"),
#         DeclareLaunchArgument("position_y", default_value="0.0"),
#         DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
#         DeclareLaunchArgument("odometry_source", default_value="world"),
#         robot_state_publisher,
#         gz_spawn_entity, gz_ros2_bridge
#     ])




import os

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch.substitutions import Command

from launch_ros.actions import Node


# ============================================================
# -------------------- BCR ROBOT GROUP ------------------------
# ============================================================

def generate_bcr_robot_group(robot_name, x, y, yaw):

    bcr_bot_path = get_package_share_directory('bcr_bot')
    twist_mux_params = os.path.join(bcr_bot_path, 'config', 'twist_mux.yaml')

    xacro_path = join(
        bcr_bot_path,
        'urdf',
        'bcr_bot.xacro'
    )

    # --------------------------------------------------------
    # ROBOT STATE PUBLISHER
    # --------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        name='robot_state_publisher',
        output='screen',

        parameters=[{
            'use_sim_time': True,
            'publish_frequency': 30.0,

            'robot_description': Command([
                'xacro ', xacro_path,

                ' robot_namespace:=', robot_name,

                ' camera_enabled:=true',
                ' stereo_camera_enabled:=false',
                ' two_d_lidar_enabled:=true',

                ' odometry_source:=world',
                ' sim_gz:=true',
            ])
        }],

        remappings=[
            ('/joint_states', f'/{robot_name}/joint_states'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),

        ]
    )

    # --------------------------------------------------------
    # SPAWN ROBOT IN GAZEBO HARMONIC
    # --------------------------------------------------------

    spawn_robot = TimerAction(
        period=3.0,

        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'{robot_name}_spawn',
                output='screen',

                arguments=[
                    '-topic',
                    f'/{robot_name}/robot_description',

                    '-name',
                    robot_name,

                    '-allow_renaming',
                    'false',

                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.28',
                    '-Y', str(yaw),
                ]
            )
        ]
    )

    # --------------------------------------------------------
    # GAZEBO HARMONIC <-> ROS 2 BRIDGE
    # --------------------------------------------------------

    bridge_topics = [

        # ----------------------------------------------------
        # DIFFERENTIAL DRIVE
        # ----------------------------------------------------

        # ROS 2 -> Gazebo
        f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

        # ----------------------------------------------------
        # ODOMETRY
        # ----------------------------------------------------

        # Gazebo -> ROS 2
        f'/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

        # ----------------------------------------------------
        # TF
        # ----------------------------------------------------

        f'/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

        # ----------------------------------------------------
        # 2D LIDAR
        # ----------------------------------------------------

        # Xacro specifies <topic>scan</topic>
        f'/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

        # ----------------------------------------------------
        # KINECT CAMERA
        # ----------------------------------------------------

        f'/{robot_name}/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image',

        f'/{robot_name}/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        f'/{robot_name}/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

        # ----------------------------------------------------
        # IMU
        # ----------------------------------------------------

        f'/{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

        # ----------------------------------------------------
        # STEREO CAMERA
        # ----------------------------------------------------

        f'/{robot_name}/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

        f'/{robot_name}/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

        f'/{robot_name}/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        f'/{robot_name}/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        # ----------------------------------------------------
        # JOINT STATES
        # ----------------------------------------------------

        f'/world/default/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_name,
        name='ros_gz_bridge',
        output='screen',

        arguments=bridge_topics,

        parameters=[{
            'use_sim_time': True
        }],

        remappings=[
            # Gazebo joint states -> ROS joint states
            (
                f'/world/default/model/{robot_name}/joint_state',
                f'/{robot_name}/joint_states'
            ),

            # # LiDAR topic remapping
            # (
            #     '/scan',
            #     f'/{robot_name}/scan'
            # ),
        ]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=robot_name,
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    # --------------------------------------------------------
    # RETURN ROBOT GROUP
    # --------------------------------------------------------

    return GroupAction([
        robot_state_publisher,
        spawn_robot,
        gz_ros2_bridge,
        twist_mux
    ])


# ============================================================
# -------------------- MAIN LAUNCH ----------------------------
# ============================================================

def generate_launch_description():

    ld = LaunchDescription()

    # --------------------------------------------------------
    # GLOBAL GAZEBO CLOCK BRIDGE
    # --------------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    ld.add_action(clock_bridge)

    # --------------------------------------------------------
    # BCR ROBOT 1
    # --------------------------------------------------------

    ld.add_action(
        generate_bcr_robot_group(
            'robot1',
            0.0,
            0.0,
            0.0
        )
    )

    # --------------------------------------------------------
    # BCR ROBOT 2
    # --------------------------------------------------------

    # Uncomment when you want to spawn another BCR robot.

    # ld.add_action(
    #     generate_bcr_robot_group(
    #         'robot2',
    #         0.0,
    #         -2.0,
    #         0.0
    #     )
    # )

    return ld
