# #!/usr/bin/python3

# from os.path import join
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration,PythonExpression
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import AppendEnvironmentVariable


# def generate_launch_description():
#     use_sim_time = LaunchConfiguration("use_sim_time", default=True)

#     bcr_bot_path = get_package_share_directory("bcr_bot")
#     world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, "worlds", "small_warehouse.sdf"))
#     gz_sim_share = get_package_share_directory("ros_gz_sim")

#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
#         launch_arguments={
#             "gz_args" : PythonExpression(["'", world_file, " -r'"])

#         }.items()
#     )

#     spawn_bcr_bot_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(join(bcr_bot_path, "launch", "bcr_bot_gz_spawn.launch.py")),
#         launch_arguments={
#             # Pass any arguments if your spawn.launch.py requires
#         }.items()
#     )

#     return LaunchDescription([

#         AppendEnvironmentVariable(
#         name='GZ_SIM_RESOURCE_PATH',
#         value=join(bcr_bot_path, "worlds")),

#         AppendEnvironmentVariable(
#         name='GZ_SIM_RESOURCE_PATH',
#         value=join(bcr_bot_path, "models")),

#         DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
#         DeclareLaunchArgument("world_file", default_value=world_file),
        
#         gz_sim, spawn_bcr_bot_node
#     ])




#!/usr/bin/env python3

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # --------------------------------------------------
    # PACKAGE PATHS
    # --------------------------------------------------

    bcr_bot_path = get_package_share_directory("bcr_bot")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    # --------------------------------------------------
    # LAUNCH ARGUMENTS
    # --------------------------------------------------

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")

    # --------------------------------------------------
    # GAZEBO WORLD
    # --------------------------------------------------

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'", world_file, " -r'"
            ])
        }.items()
    )

    # --------------------------------------------------
    # BCR ROBOT SPAWN
    # --------------------------------------------------

    spawn_bcr_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                bcr_bot_path,
                "launch",
                "bcr_bot_gz_spawn.launch.py"
            )
        )
    )

    # Wait for Gazebo to start before spawning the robot
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_bcr_bot]
    )

    # --------------------------------------------------
    # LAUNCH DESCRIPTION
    # --------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock"
        ),

        DeclareLaunchArgument(
            "world_file",
            default_value=join(
                bcr_bot_path,
                "worlds",
                "small_warehouse.sdf"
            ),
            description="Gazebo world file"
        ),

        # Gazebo resource paths
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=join(bcr_bot_path, "worlds")
        ),

        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=join(bcr_bot_path, "models")
        ),

        # Start Gazebo
        gz_sim,

        # Spawn BCR after Gazebo starts
        delayed_spawn,
    ])
