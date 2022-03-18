cmake_minimum_required(VERSION 3.0.2)
project(dwa_demo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  costmap_2d
  dwa_local_planner
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_demo
  CATKIN_DEPENDS roscpp std_msgs tf2_ros costmap_2d dwa_local_planner
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})
add_dependencies(demo ${catkin_EXPORTED_TARGETS})

