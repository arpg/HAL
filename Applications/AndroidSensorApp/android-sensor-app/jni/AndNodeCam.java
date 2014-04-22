
public class AndNodeCam {

	static {
		System.loadLibrary("andnodecam");
	}
	
	private native void sendData(byte[] bytes);
	
	private native void Initialize();
	
	public static void sendCamData( byte[] bytes )
	{
		new AndNodeCam().sendData(bytes);
	}
	
}
