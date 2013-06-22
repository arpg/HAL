project(AndroidLogger) 
cmake_minimum_required(VERSION 2.8)

set( CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS}" )

# Generate ant build scripts for making APK
execute_process(
    COMMAND android update project --name ${PROJECT_NAME} --path ${PROJECT_SOURCE_DIR} --target android-17 --subprojects
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add application c library
add_subdirectory(jni)

# clean generated files
add_custom_target( clean-src
    COMMAND rm -rf bin
    COMMAND rm -rf libs
    COMMAND rm build.xml
    COMMAND rm local.properties
    COMMAND rm project.properties
    COMMAND rm proguard-project.txt
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

set(APK_NAME "${PROJECT_NAME}-debug.apk")

# Package APK
# ant release
add_custom_command(
    OUTPUT ${PROJECT_SOURCE_DIR}/bin/${APK_NAME}
    COMMAND ant debug
    DEPENDS ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Install to device
# adb install -r bin/AndroidLogger-debug.apk
add_custom_target( push
    COMMAND adb install -r bin/${APK_NAME}
    DEPENDS ${PROJECT_SOURCE_DIR}/bin/${APK_NAME}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

# install and run on device
add_custom_target( run
    COMMAND adb shell am start -n hal.apps.android/android.app.NativeActivity
    DEPENDS push
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

# install and run on device, piping logcat to terminal
add_custom_target( debug
    COMMAND adb logcat -c && adb shell am start -n hal.apps.android/android.app.NativeActivity && adb logcat
    DEPENDS push
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
