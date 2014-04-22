package edu.gwu.rpg.androidnodecam;

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

public class MainActivity extends Activity implements SurfaceHolder.Callback {

	 Camera mCamera;
	 SurfaceView mPreview;
	 AndNodeCam anc;
	 String ip_address;
	
	 
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);        

        anc = new AndNodeCam();
        anc.InitANC( "192.168.137.1", 5555 );
        
        mPreview = (SurfaceView)findViewById(R.id.preview);
        mPreview.getHolder().addCallback(this);
        mPreview.getHolder().setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
        
        mCamera = Camera.open();
//        Parameters p = mCamera.getParameters();
//        p.set("orientation", "portrait");
//        mCamera.setParameters(p);
        mCamera.setDisplayOrientation(90);
        
        mCamera.setPreviewCallback( new Camera.PreviewCallback() {
			
			@Override
			public void onPreviewFrame(byte[] data, Camera camera) {
				// TODO Auto-generated method stub
				Camera.Parameters parameters = camera.getParameters();

                int width = parameters.getPreviewSize().width;
                int height = parameters.getPreviewSize().height;
                byte[] toSend = new byte[width*height];
                System.arraycopy(data, 0, toSend, 0, width*height);
				anc.sendCamData(toSend);
//				try {
//					Thread.sleep(100);
//				} catch (InterruptedException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
			}
		});
    }


    @Override
    public void onPause() {
    	super.onPause();
        mCamera.stopPreview();
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
	public void onDestroy() {
		super.onDestroy();
        mCamera.release();
	}
	
	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		// TODO Auto-generated method stub
		try {
            mCamera.setPreviewDisplay(mPreview.getHolder());
        } catch (Exception e) {
            e.printStackTrace();
        }
	}


	@Override
	public void surfaceDestroyed(SurfaceHolder holder) {
		// TODO Auto-generated method stub
		
	}
    
}
