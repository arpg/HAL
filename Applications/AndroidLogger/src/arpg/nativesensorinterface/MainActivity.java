package arpg.androidlogger;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.app.Activity;
import android.content.Context;
import android.hardware.Camera;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.opengl.GLSurfaceView;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.widget.TextView;
import android.util.Log;

public class MainActivity extends Activity {
    private NativeCameraInterface mCamera;
    private NativeSensorInterface mNativeInterface;
    private NativeOpenGLRenderer mRenderer;
    private GLSurfaceView mGlSurface;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mRenderer = new NativeOpenGLRenderer();
        mGlSurface = (GLSurfaceView)findViewById(R.id.canvas);
        mGlSurface.setRenderer(mRenderer);

        mNativeInterface = new NativeSensorInterface();
        mNativeInterface.initialize(this);
        TextureView texture = (TextureView)findViewById(R.id.preview);
        mCamera = new NativeCameraInterface(mNativeInterface, texture);

        mNativeInterface.setTextViews((TextView)findViewById(R.id.gps_text),
                                      (TextView)findViewById(R.id.gyro_text),
                                      (TextView)findViewById(R.id.accel_text),
                                      (TextView)findViewById(R.id.image_text));
    }

    @Override
    public void onPause() {
        super.onPause();
        mCamera.stop();
        mNativeInterface.stop();
        mRenderer.stop();
        mGlSurface.onPause();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mCamera.close();
    }

    static {
        System.loadLibrary("gnustl_shared");
        System.loadLibrary("ARPGNativeInterface");
    }
}
