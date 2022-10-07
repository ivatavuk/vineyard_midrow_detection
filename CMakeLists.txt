cmake_minimum_required(VERSION 3.0.2)
project(vineyard_midrow_detection)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

generate_dynamic_reconfigure_options(
  config/MidrowDetection.cfg)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

##-------------------------------------------------

add_executable(vineyard_midrow_detection 
  src/vineyard_midrow_detection.cpp
  src/PclMidRowDetection.cpp)

target_link_libraries(vineyard_midrow_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(vineyard_midrow_detection 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)