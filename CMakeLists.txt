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
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS sgtdv_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/PathTrackingNODE.cpp src/PathTracking.cpp src/PathTrackingSynch.cpp src/TrackingAlgorithms.cpp)

target_link_libraries(path_tracking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


