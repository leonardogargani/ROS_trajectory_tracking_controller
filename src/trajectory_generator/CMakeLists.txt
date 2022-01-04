cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)


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

# REMEMBER TO MODIFY THE CMAKELISTS.TXT OF THE SERVICE CLIENT (look at my old Robotics project)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(eight_traj_gen src/eight_traj_gen.cpp)
target_link_libraries(eight_traj_gen ${catkin_LIBRARIES})
