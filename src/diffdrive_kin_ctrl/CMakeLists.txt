cmake_minimum_required(VERSION 3.0.2)
project(diffdrive_kin_ctrl)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


add_service_files(
	FILES
	GenerateDesiredPathService.srv
)


generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diffdrive_kin_ctrl
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(diffdrive_kin_trajctrl src/diffdrive_kin_trajctrl.cpp src/diffdrive_kin_trajctrl_node.cpp src/diffdrive_kin_fblin.cpp)
add_executable(eight_traj_gen src/eight_traj_gen.cpp)


target_link_libraries(diffdrive_kin_trajctrl ${catkin_LIBRARIES})
target_link_libraries(eight_traj_gen ${catkin_LIBRARIES})


add_dependencies(diffdrive_kin_trajctrl ${catkin_EXPORTED_TARGETS})
add_dependencies(eight_traj_gen ${catkin_EXPORTED_TARGETS})