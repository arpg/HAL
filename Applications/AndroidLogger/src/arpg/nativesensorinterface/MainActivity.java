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
import android.widget.ToggleButton;
import android.widget.TextView;
import android.widget.Toast;
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

        mNativeInterface.setTextViews((TextView)findViewById(R.id.gps_text),
                                      (TextView)findViewById(R.id.gyro_text),
                                      (TextView)findViewById(R.id.accel_text),
                                      (TextView)findViewById(R.id.image_text),
                                      (TextView)findViewById(R.id.log_text),
                                      (TextView)findViewById(R.id.mag_text));

        final Context ctx = this;
        mPlayButton = (Button)findViewById(R.id.play_button);
        final ToggleButton shouldMoveButton =
            (ToggleButton)findViewById(R.id.move_to_sd);
        final ToggleButton isPeanutButton =
            (ToggleButton)findViewById(R.id.is_peanut);
        mPlayButton.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    mNativeInterface.
                        setIsLogging(!mNativeInterface.isLogging());
                    if (mNativeInterface.isLogging()) {

                        int width = mCamera.getWidth();
                        int height = mCamera.getHeight();
                        if (isPeanutButton.isChecked()) {
                            height = 1168;
                            width = 1280;
                        }

                        mNativeInterface.initialize(ctx, width, height,
                                                    isPeanutButton.isChecked());

                        mPlayButton.setBackgroundColor(Color.RED);
                        mPlayButton.setText(getString(R.string.stop_record));
                    } else {
                        Toast.makeText(ctx, "AndroidLogger stopping.",
                                       Toast.LENGTH_SHORT).show();
                        mNativeInterface.stop(shouldMoveButton.isChecked());
                        Toast.makeText(ctx, "AndroidLogger finished.",
                                       Toast.LENGTH_SHORT).show();

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
