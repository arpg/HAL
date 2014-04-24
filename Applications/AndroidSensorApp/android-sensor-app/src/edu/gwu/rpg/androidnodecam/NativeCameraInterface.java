package edu.gwu.rpg.nativesensorinterface;

import android.view.SurfaceHolder;
import android.content.Context;
import android.hardware.Camera;

public class NativeCameraInterface implements SurfaceHolder.Callback {
    private Camera mCamera;
    private NativeSensorInterface mNativeInterface;

    public NativeCameraInterface(NativeSensorInterface nativeInterface) {
        mNativeInterface = nativeInterface;
        mCamera = Camera.open();
        mCamera.setDisplayOrientation(90);

        mCamera.setPreviewCallback(new Camera.PreviewCallback() {
                @Override
                public void onPreviewFrame(byte[] data, Camera camera) {
                    mNativeInterface.PostImage(data);
                }
            });
    }

    public void stop() {
        mCamera.stopPreview();
    }

    public void close() {
        mCamera.release();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width,
                               int height) {
        Camera.Parameters params = mCamera.getParameters();
        params.setPreviewSize(640, 480);
        mCamera.setParameters(params);
        mCamera.startPreview();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        try {
            mCamera.setPreviewDisplay(holder);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {}
}
