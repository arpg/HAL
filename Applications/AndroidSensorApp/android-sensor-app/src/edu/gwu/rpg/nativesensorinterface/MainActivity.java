package edu.gwu.rpg.nativesensorinterface;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.app.Activity;
import android.content.Context;
import android.hardware.Camera;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public class MainActivity extends Activity {

    private SurfaceView mPreview;
    private NativeCameraInterface mCamera;
    private NativeSensorInterface mNativeInterface;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mNativeInterface = new NativeSensorInterface();
        mCamera = new NativeCameraInterface(mNativeInterface);
        mPreview = (SurfaceView)findViewById(R.id.preview);
        mPreview.getHolder().addCallback(mCamera);
        mPreview.getHolder().setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
    }

    @Override
    public void onPause() {
        super.onPause();
        mCamera.stop();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mCamera.close();
    }
}
