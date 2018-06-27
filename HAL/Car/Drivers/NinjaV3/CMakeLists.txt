set(BUILD_NinjaV3 false CACHE BOOL force)

if(BUILD_NinjaV3)

    add_to_hal_sources(NinjaV3CarDriver.h NinjaV3CarDriver.cpp NinjaV3CarFactory.cpp ComDriver.h)

endif()
