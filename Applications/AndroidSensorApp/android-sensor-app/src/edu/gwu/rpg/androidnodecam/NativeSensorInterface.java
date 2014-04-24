package edu.gwu.rpg.nativesensorinterface;

/** The class for handling all JNI data interchange
 *
 * Naming convention:
 *  - Java calls are CamelCase
 *  - Native functions are lower_case_with_underlines
 *  - Data methods are Post* (Image, Imu, etc.)
 */
public class NativeSensorInterface {
    private native void post_image(byte[] bytes);
    private native void initialize();

    public void Initialize() {
        initialize();
    }

    public void PostImage(byte[] bytes) {
        post_image(bytes);
    }

    static {
        System.loadLibrary("gnustl_shared");
        System.loadLibrary("opencv_java");
        System.loadLibrary("pbmsgs");
        System.loadLibrary("miniglog");
        System.loadLibrary("node");
        System.loadLibrary("calibu");
        System.loadLibrary("hal");
        System.loadLibrary("NativeSensorInterface");
    }
}
