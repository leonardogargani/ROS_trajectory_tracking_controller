cmake_minimum_required(VERSION 3.0.2)
project(unicycle_kin_sim)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unicycle_kin_sim
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(unicycle_kin_sim src/unicycle_kin_sim.cpp src/unicycle_kin_sim_node.cpp src/unicycle_kin_ode.cpp)
target_link_libraries(unicycle_kin_sim ${catkin_LIBRARIES})
