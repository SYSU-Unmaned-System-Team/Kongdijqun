cmake_minimum_required(VERSION 2.8.3)
project(prometheus_case2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       laser_geometry
       std_msgs
	   sensor_msgs
	   geometry_msgs
	   nav_msgs
	   pcl_ros
       visualization_msgs
	   prometheus_msgs
	)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES 
	#  DEPENDS system_lib
)

include_directories(
       SYSTEM
       include
	   ${PROJECT_SOURCE_DIR}/include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
	   ${PROJECT_SOURCE_DIR}/../../common/include
     )
link_directories(${PCL_LIBRARY_DIRS})


add_executable(case2_main
	src/global_planner_node.cpp
	src/global_planner.cpp
	src/A_star.cpp
	src/occupy_map.cpp
	src/tools.cpp
	)

target_link_libraries(case2_main
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
	)

add_executable(case2_to_groundstation src/case2_to_groundstation.cpp)
add_dependencies(case2_to_groundstation prometheus_case2_gencpp)
target_link_libraries(case2_to_groundstation ${catkin_LIBRARIES})

add_executable(case2_from_groundstation src/case2_from_groundstation.cpp)
add_dependencies(case2_from_groundstation prometheus_case2_gencpp)
target_link_libraries(case2_from_groundstation ${catkin_LIBRARIES})
