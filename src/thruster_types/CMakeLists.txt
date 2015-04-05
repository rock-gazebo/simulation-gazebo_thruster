find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs thruster_types.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(thruster_types SHARED ${PROTO_SRCS})
target_link_libraries(thruster_types ${PROTOBUF_LIBRARY})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/thruster_types.pb.h DESTINATION include/thruster_types)

link_directories( ${CMAKE_CURRENT_BINARY_DIR}/thruster_types )
install(TARGETS thruster_types
        LIBRARY DESTINATION lib)
