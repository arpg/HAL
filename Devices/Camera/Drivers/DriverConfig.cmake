# Saving some intersting macros that are a work in progress
set( DRIVER_LIST "" )

#
macro( add_driver DRIVER_DIR )
    include( ${DRIVER_DIR}/DriverConfig.cmake )
    if( ${BUILD_DRIVER} )
        add_subdirectory( ${DRIVER_DIR} )
        set( USE_${DRIVER_NAME} 
"// AUTOMAGICALLY GENERATED:
#ifdef USE_${DRIVER_NAME}
#include \"RPG/Devices/Camera/Drivers/${DRIVER_DIR}/${DRIVER_HEADER}\"
CameraDriverRegisteryEntry<${DRIVER_CLASS}> _${DRIVER_NAME}Reg( \"${DRIVER_NAME}\" );
#endif"
)
        add_definitions( -DUSE_${DRIVER_NAME} )
        list( APPEND DRIVER_LIST ${USE_${DRIVER_NAME}} )
        message( STATUS "out '${DRIVER_LIST}" )
    endif()
endmacro()

# Output driver list so we compile known drivers in:
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/DriverList.h.in
                        ${CMAKE_CURRENT_SOURCE_DIR}/test.h IMMEDIATE @ONLY )


