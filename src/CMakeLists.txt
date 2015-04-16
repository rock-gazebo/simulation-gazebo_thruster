find_package(Boost 1.40 COMPONENTS system REQUIRED)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(msgs_SRCS msgs_HDR msgs.proto)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rock_library(gazebo_thruster
    SOURCES GazeboThruster.cpp ${msgs_SRCS}
    HEADERS ${msgs_HDR}
    DEPS_PLAIN Boost
    DEPS_PKGCONFIG gazebo sdformat protobuf)

install(FILES msgs.proto
    DESTINATION include/${PROJECT_NAME})
