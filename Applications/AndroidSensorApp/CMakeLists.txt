project(AndroidSensorApp)

# Declare apk w/
#   - Library dependencies
#   - JNI Directory
#   - JNI Sources
# Generate Android.mk in build dir
# - Add each dependent library's full path
# - Add all include directories
# - Add compile flags for C, CXX
# - Add JNI sources
# Run ndk-build in Android.mk dir
add_subdirectory(AndCam)