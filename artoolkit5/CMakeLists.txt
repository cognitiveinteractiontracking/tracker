cmake_minimum_required(VERSION 2.8.3)
project(artoolkit5)

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
if (ENV{ARTOOLKIT5_ROOT} STREQUAL "")
  message(WARNING "ARTOOLKIT5_ROOT is not defined, setting variable to /opt/artoolkit5")
  set(ENV{ARTOOLKIT5_ROOT} "/opt/artoolkit5")
endif (ENV{ARTOOLKIT5_ROOT} STREQUAL "")
find_package(ARToolKit5 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT 10.0 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GSTR gstreamer-0.10)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES genicam arghandler
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs tf
# DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDES}
  ${ARTOOLKIT_INCLUDE_DIR}
  ${GSTR_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(artoolkit5_node src/artoolkit5_node.cpp)

target_link_libraries(artoolkit5_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GLUT_LIBRARY}
  ${GSTR_LIBRARIES}
  ${ARTOOLKIT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  jpeg
)
