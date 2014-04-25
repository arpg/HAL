package arpg.androidlogger;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.hardware.Camera;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.util.Log;

public class MainActivity extends Activity {
    private NativeCameraInterface mCamera;
    private NativeSensorInterface mNativeInterface;
    private NativeOpenGLRenderer mRenderer;
    private Button mPlayButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mRenderer = new NativeOpenGLRenderer();
        mNativeInterface = new NativeSensorInterface();
        TextureView texture = (TextureView)findViewById(R.id.preview);
        mCamera = new NativeCameraInterface(mNativeInterface, texture);

        mNativeInterface.initialize(this, mCamera.getWidth(),
                                    mCamera.getHeight());

        mNativeInterface.setTextViews((TextView)findViewById(R.id.gps_text),
                                      (TextView)findViewById(R.id.gyro_text),
                                      (TextView)findViewById(R.id.accel_text),
                                      (TextView)findViewById(R.id.image_text),
                                      (TextView)findViewById(R.id.log_text));

        mPlayButton = (Button)findViewById(R.id.play_button);
        mPlayButton.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    mNativeInterface.
                        setIsLogging(!mNativeInterface.isLogging());
                    if (mNativeInterface.isLogging()) {
                        mPlayButton.setBackgroundColor(Color.RED);
                        mPlayButton.setText(getString(R.string.stop_record));
                    } else {
                        mPlayButton.setBackgroundColor(Color.GREEN);
                        mPlayButton.setText(getString(R.string.record));
                    }
                }
            });
    }

    @Override
    public void onPause() {
        super.onPause();
        mCamera.stop();
        mNativeInterface.stop();
        mRenderer.stop();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mCamera.close();
    }

    static {
        System.loadLibrary("gnustl_shared");
        System.loadLibrary("miniglog");
        System.loadLibrary("opencv_java");
        System.loadLibrary("pbmsgs");
        System.loadLibrary("ARPGNativeInterface");
    }
}
