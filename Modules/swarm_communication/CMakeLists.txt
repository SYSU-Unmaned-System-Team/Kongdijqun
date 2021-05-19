cmake_minimum_required(VERSION 3.0.2)
project(prometheus_swarm_communication)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  prometheus_msgs
  tf2_eigen
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against

add_executable(ground_station_bridge src/ground_station_bridge.cpp)
target_link_libraries(ground_station_bridge ${catkin_LIBRARIES})


add_executable(swarm_multicast_bridge
  src/swarm_multicast_bridge.cpp
)
add_dependencies(swarm_multicast_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_multicast_bridge
  ${catkin_LIBRARIES}
)

add_executable(swarm_end2end_bridge
  src/swarm_end2end_bridge.cpp
)
add_dependencies(swarm_end2end_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_end2end_bridge
  ${catkin_LIBRARIES}
)