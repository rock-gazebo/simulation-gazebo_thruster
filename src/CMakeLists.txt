find_package(Boost 1.40 COMPONENTS system REQUIRED)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(thruster_types_SRCS thruster_types_HDR thruster_types.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rock_library(gazebo_thruster
    SOURCES GazeboThruster.cpp ${thruster_types_SRCS}
    HEADERS ${thruster_types_HDR}
    DEPS_PLAIN Boost
    DEPS_PKGCONFIG gazebo sdformat protobuf)

