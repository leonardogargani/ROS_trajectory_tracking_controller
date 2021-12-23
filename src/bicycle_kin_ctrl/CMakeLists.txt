cmake_minimum_required(VERSION 3.0.2)
project(bicycle_kin_ctrl)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bicycle_kin_ctrl
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(bicycle_kin_linctrl src/bicycle_kin_linctrl.cpp src/bicycle_kin_linctrl_node.cpp src/bicycle_kin_fblin.cpp)
target_link_libraries(bicycle_kin_linctrl ${catkin_LIBRARIES})

add_executable(bicycle_kin_trajctrl src/bicycle_kin_trajctrl.cpp src/bicycle_kin_trajctrl_node.cpp src/bicycle_kin_fblin.cpp)
target_link_libraries(bicycle_kin_trajctrl ${catkin_LIBRARIES})
