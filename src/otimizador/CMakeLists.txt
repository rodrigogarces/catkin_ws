cmake_minimum_required(VERSION 2.8.3)
project(otimizador)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  dynamic_reconfigure
  std_msgs
  genmsg
  tf
  costmap_2d
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  sensor_msgs
  filters
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

add_library(Node src/Node.cpp)
target_link_libraries(Node ${catkin_LIBRARIES})

add_library(AStar src/AStar.cpp)
target_link_libraries(AStar ${catkin_LIBRARIES} Node)

add_library(Firefly src/Firefly.cpp)
target_link_libraries(Firefly ${catkin_LIBRARIES})

add_library(OptimizationMethods src/OptimizationMethods.cpp)
target_link_libraries(OptimizationMethods ${catkin_LIBRARIES})

add_executable(otimizador src/otimizador.cpp)
target_link_libraries(otimizador ${catkin_LIBRARIES} Firefly AStar OptimizationMethods)
#add_dependencies(otimizador otimizador_generate_target_cpp)


