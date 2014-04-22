package edu.gwu.rpg.androidnodecam;

public class AndNodeCam {

	static {
		System.loadLibrary("gnustl_shared");
		System.loadLibrary("opencv_java");
        System.loadLibrary("pbmsgs");
		System.loadLibrary("miniglog");
		System.loadLibrary("node");
		System.loadLibrary("calibu");
		System.loadLibrary("hal");
		System.loadLibrary("AndCam");
		System.loadLibrary("AndNodeCam");
	}

	private native void sendData(byte[] bytes);

	private native void Initialize( String IP, int port);

	public void InitANC( String ip_address, int port )
	{
		Initialize( ip_address, port);
	}

	public void sendCamData( byte[] bytes )
	{ // 0x5305e2b4
		sendData(bytes);
	}

}
