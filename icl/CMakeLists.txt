cmake_minimum_required(VERSION 2.8.3)
project(icl)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_PREFIX_PATH "/opt/icl9.12.1/")

add_definitions(-std=c++11 -fPIC)

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
find_package(ICL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs tf cv_bridge
# DEPENDS system_lib
)

include_directories(
  ${OpenCV_INCLUDES}
  ${catkin_INCLUDE_DIRS}
  ${ICL_INCLUDE_DIRS}
)

add_executable(icl_node src/icl_node.cpp)

target_link_libraries(icl_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ICL_LIBRARIES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
