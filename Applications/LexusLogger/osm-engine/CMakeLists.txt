cmake_minimum_required( VERSION 2.8 )
project( osm-engine )


#list( APPEND LINK_LIBS  ${PROTOBUF_LIBRARIES} )

find_package(Protobuf REQUIRED)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include
                     ${PROTOBUF_INCLUDE_DIR}
                     ${EIGEN3_INCLUDE_DIR} )

#set( _PROTO_FILES include/osm.proto )
#PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS ${_PROTO_FILES} )

set( SRCS
     #${PROTO_SRCS}
     src/OpenStreetMap.cpp
     src/TinyXML.cpp
     src/osm.pb.cc
    )

set ( HDRS
     #${PROTO_HDRS}
     include/OpenStreetMap.h
     include/TinyXML.h
     include/osm.pb.h
    )

add_library( osm-engine ${SRCS} ${HDRS} )
target_link_libraries( osm-engine ${PROTOBUF_LIBRARIES} curl)


