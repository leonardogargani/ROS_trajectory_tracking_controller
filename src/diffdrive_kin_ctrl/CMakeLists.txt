cmake_minimum_required(VERSION 3.0.2)
project(diffdrive_kin_ctrl)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diffdrive_kin_ctrl
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(diffdrive_kin_trajctrl src/diffdrive_kin_trajctrl.cpp src/diffdrive_kin_trajctrl_node.cpp src/diffdrive_kin_fblin.cpp)
target_link_libraries(diffdrive_kin_trajctrl ${catkin_LIBRARIES})
