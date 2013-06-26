# Configure build environment to automatically generate APK's instead of executables.
if(ANDROID)
    # Reset output directories to be in binary folder (rather than source)
    set(LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_CURRENT_BINARY_DIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH_ROOT}/libs/${ANDROID_NDK_ABI_NAME})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH_ROOT}/libs/${ANDROID_NDK_ABI_NAME})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH_ROOT}/bin/${ANDROID_NDK_ABI_NAME})
    
    macro( create_android_manifest_xml filename library_so_name package_name activity_name)
        file( WRITE ${filename}
"<?xml version=\"1.0\" encoding=\"utf-8\"?>
<!-- BEGIN_INCLUDE(manifest) -->
<manifest xmlns:android=\"http://schemas.android.com/apk/res/android\"
        package=\"${package_name}\"
        android:versionCode=\"1\"
        android:versionName=\"1.0\">

    <!-- This is the platform API where NativeActivity was introduced. -->
    <uses-sdk android:minSdkVersion=\"14\" />
    <uses-feature android:name=\"android.hardware.camera\" />
    <uses-permission android:name=\"android.permission.CAMERA\"/>
    <uses-permission android:name=\"android.permission.WRITE_EXTERNAL_STORAGE\"/>
    <uses-permission android:name=\"android.permission.READ_EXTERNAL_STORAGE\"/>

    <!-- This .apk has no Java code itself, so set hasCode to false. -->
    <application android:label=\"${activity_name}\" android:hasCode=\"false\">

        <!-- Our activity is the built-in NativeActivity framework class.
             This will take care of integrating with our NDK code. -->
        <activity android:name=\"android.app.NativeActivity\"
                android:label=\"${activity_name}\"
                android:configChanges=\"orientation|keyboardHidden\">
            <!-- Tell NativeActivity the name of our .so -->
            <meta-data android:name=\"android.app.lib_name\"
                    android:value=\"${library_so_name}\" />
            <intent-filter>
                <action android:name=\"android.intent.action.MAIN\" />
                <category android:name=\"android.intent.category.LAUNCHER\" />
            </intent-filter>
        </activity>
    </application>

</manifest> 
<!-- END_INCLUDE(manifest) -->"    )        
    endmacro()

    macro( android_update android_project_name)
        # Generate ant build scripts for making APK
        execute_process(
            COMMAND android update project --name ${android_project_name} --path . --target android-17 --subprojects
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        )
    endmacro()

    # Override add_executable to build android .so instead!
    macro( add_executable prog_name)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libs/${ANDROID_NDK_ABI_NAME})
        add_library( ${prog_name} SHARED ${ARGN} )

        set(ANDROID_PACKAGE_NAME "edu.gw.robotics.${prog_name}")
        create_android_manifest_xml(
            "${CMAKE_CURRENT_BINARY_DIR}/AndroidManifest.xml" ${prog_name}
            "${ANDROID_PACKAGE_NAME}" "${prog_name}"
        )

        # Generate ant build system for apk
        android_update( ${prog_name} )

        # Target to invoke ant build system for apk
        set( APK_FILE "${CMAKE_CURRENT_BINARY_DIR}/bin/${prog_name}-debug.apk" )
        add_custom_command(
            OUTPUT ${APK_FILE}
            COMMAND ant debug
            DEPENDS ${prog_name}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        )

        # Target to install on device
        add_custom_target( apk
            DEPENDS ${APK_FILE}
        )

        # Target to install on device
        add_custom_target( push
            COMMAND adb install -r ${APK_FILE}
            DEPENDS ${APK_FILE}
        )

        # install and run on device
        add_custom_target( run
            COMMAND adb shell am start -n ${ANDROID_PACKAGE_NAME}/android.app.NativeActivity
            DEPENDS push
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endmacro()

    
    ## clean generated files
    #add_custom_target( clean-src
    #    COMMAND rm -rf bin
    #    COMMAND rm -rf libs
    #    COMMAND rm build.xml
    #    COMMAND rm local.properties
    #    COMMAND rm project.properties
    #    COMMAND rm proguard-project.txt
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #)
    
    #set(APK_NAME "${PROJECT_NAME}-debug.apk")
    
    ## Package APK
    ## ant release
    #add_custom_command(
    #    OUTPUT ${PROJECT_SOURCE_DIR}/bin/${APK_NAME}
    #    COMMAND ant debug
    #    DEPENDS native-activity
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #)
    
    ## Install to device
    ## adb install -r bin/native-activity-debug.apk
    #add_custom_target( push
    #    COMMAND adb install -r bin/${APK_NAME}
    #    DEPENDS ${PROJECT_SOURCE_DIR}/bin/${APK_NAME}
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #)
    
    ## install and run on device
    #add_custom_target( run
    #    COMMAND adb shell am start -n com.example.native_activity/android.app.NativeActivity
    #    DEPENDS push
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #)
    
    ## install and run on device, piping logcat to terminal
    #add_custom_target( debug
    #    COMMAND adb logcat -c && adb shell am start -n com.example.native_activity/android.app.NativeActivity && adb logcat
    #    DEPENDS push
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #)
endif()
