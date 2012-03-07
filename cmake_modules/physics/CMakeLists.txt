cmake_minimum_required( VERSION 2.8 )

project( Mochaccino )

# Mochaccino Files
set( HDRS Physics.h)
set( SRCS Physics.cpp)


add_library( physics ${HDRS} ${SRCS})


