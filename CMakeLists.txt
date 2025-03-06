cmake_minimum_required(VERSION 3.0.2)
project(path_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sgtdv_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS sgtdv_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../
)

add_executable(
  ${PROJECT_NAME} 
  src/path_tracking_node.cpp 
  src/path_tracking_ros.cpp 
  src/path_tracking.cpp
  src/speed_control.cpp
  src/steering_control.cpp)

target_link_libraries(path_tracking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


