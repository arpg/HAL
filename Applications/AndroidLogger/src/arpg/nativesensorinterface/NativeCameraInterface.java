package arpg.androidlogger;

import java.io.IOException;

import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.Camera.Parameters;
import android.os.SystemClock;
import android.view.TextureView;

public class NativeCameraInterface
    implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private NativeSensorInterface mNativeInterface;
    private TextureView mTextureView;
    private long mTimestamp;
    private boolean mHasInitialImage;
    private long mInitialTimestamp, mRealImageTime;

    public NativeCameraInterface(NativeSensorInterface nativeInterface,
                                 TextureView textureView) {
        mHasInitialImage = false;
        mTimestamp = 0;
        mNativeInterface = nativeInterface;
        mTextureView = textureView;
        mTextureView.setSurfaceTextureListener(this);

        mCamera = Camera.open();
        mCamera.setPreviewCallback(new Camera.PreviewCallback() {
                @Override
                public void onPreviewFrame(byte[] data, Camera camera) {
                    mNativeInterface.postImage(mTimestamp, data);
                }
            });
        Camera.Parameters params = mCamera.getParameters();
        params.setPreviewSize(640, 480);
        mCamera.setParameters(params);
    }

    int getWidth() {
        return mCamera.getParameters().getPreviewSize().width;
    }

    int getHeight() {
        return mCamera.getParameters().getPreviewSize().height;
    }

    public void stop() {
        mCamera.stopPreview();
    }

    public void close() {
        mCamera.release();
    }

    public void flash_on() {
        Parameters p = mCamera.getParameters();
        p.setFlashMode(Parameters.FLASH_MODE_TORCH);
        mCamera.setParameters(p); 
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface,
                                          int width, int height) {
        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface,
                                            int width, int height) {}

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        return true;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        if (!mHasInitialImage) {
            mHasInitialImage = true;
            mInitialTimestamp = surface.getTimestamp();
            mRealImageTime = SystemClock.elapsedRealtime();
        }
        mTimestamp =
            surface.getTimestamp() - mInitialTimestamp + mRealImageTime;
    }
}
