cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/thruster_types)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/thruster_types)
add_subdirectory(thruster_types)

rock_library(gazebo_thruster
    SOURCES GazeboThruster.cpp
    HEADERS GazeboThruster.hpp
#    DEPS thruster_types_msgs
    DEPS_PKGCONFIG gazebo protobuf)
