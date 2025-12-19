cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz maps route_graphs
    DESTINATION share/${PROJECT_NAME}/
)

install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)
         
         
install(PROGRAMS 
        scripts/initial_pose_publisher.py
        scripts/multi_robot_obstacle_node.py
        scripts/remapper.py
        scripts/goal_relay.py
        scripts/collision_detector.py
        scripts/controller_switch_node.py
        scripts/goal_cancel_node.py
        scripts/start_stop.py
        DESTINATION lib/${PROJECT_NAME})


ament_package()
