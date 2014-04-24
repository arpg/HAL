package edu.gwu.rpg.nativesensorinterface;

public class NativeSensorInterface {
    private native void post_image(byte[] bytes);
    private native void initialize(String IP, int port);

    public void Init(String ip_address, int port) {
        initialize(ip_address, port);
    }

    public void PostImage( byte[] bytes ) {
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
        System.loadLibrary("AndCam");
        System.loadLibrary("NativeSensorInterface");
    }
}
