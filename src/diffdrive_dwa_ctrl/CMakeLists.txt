cmake_minimum_required(VERSION 3.0.2)
project(diffdrive_dwa_ctrl)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf2_ros
    costmap_2d
    diffdrive_kin_ctrl
    message_generation
    base_local_planner
    dwa_local_planner
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES diffdrive_dwa_ctrl
    CATKIN_DEPENDS roscpp std_msgs tf2_ros costmap_2d diffdrive_kin_ctrl base_local_planner dwa_local_planner
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(diffdrive_dwa_trajctrl src/diffdrive_dwa_trajctrl.cpp src/diffdrive_dwa_trajctrl_node.cpp)
target_link_libraries(diffdrive_dwa_trajctrl ${catkin_LIBRARIES})
add_dependencies(diffdrive_dwa_trajctrl ${catkin_EXPORTED_TARGETS})

add_executable(odom_to_baselink_tf src/odom_to_baselink_tf.cpp src/odom_to_baselink_tf_node.cpp)
target_link_libraries(odom_to_baselink_tf ${catkin_LIBRARIES})
add_dependencies(odom_to_baselink_tf ${catkin_EXPORTED_TARGETS})
