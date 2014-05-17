package arpg.nativesensorinterface;

import android.opengl.GLSurfaceView;
import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;

public class NativeOpenGLRenderer implements GLSurfaceView.Renderer {
    private native void initialize();
    private native void resize(int width, int height);
    private native void draw_frame();
    private native void finish();

    public void stop() {
        finish();
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        initialize();
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        draw_frame();
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        resize(width, height);
    }
}
