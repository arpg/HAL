cmake_minimum_required( VERSION 2.8 )

find_package( Node REQUIRED )

include_directories( ${Node_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} )
link_libraries( ${Node_LIBRARIES} )

include_directories( ${CMAKE_BINARY_DIR}/Examples )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS ExampleMessage.proto )

add_executable( Node Node.cpp ${PROTO_SRCS} )
add_dependencies( Node node )

add_executable( Node1 Node1.cpp ${PROTO_SRCS} )
add_dependencies( Node1 node )

add_executable( Node2 Node2.cpp ${PROTO_SRCS} )
add_dependencies( Node2  node )


