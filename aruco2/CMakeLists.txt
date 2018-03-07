cmake_minimum_required(VERSION 2.8.3)
project(aruco2)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV 3.1 REQUIRED)
find_package(aruco REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs tf
# DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDES}
  ${aruco_INCLUDE_DIRS}
)

add_executable(aruco2_node src/aruco2_node.cpp)

target_link_libraries(aruco2_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${aruco_LIBS}
)
