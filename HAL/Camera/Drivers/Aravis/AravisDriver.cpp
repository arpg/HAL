#include "AravisDriver.h"



namespace hal {

  AravisDriver::AravisDriver(string m_serial )
    : serialNumber(m_serial)
  {
    
    Start();
  }
  
  AravisDriver::~AravisDriver()
  {
    Stop();

  }

  
  size_t AravisDriver::NumChannels() const
  {
    return 1; 
  }

  size_t AravisDriver::Width( size_t idx) const
  {
    return width;
  }

  size_t AravisDriver::Height( size_t idx) const
  {
    return height;
  }

  void AravisDriver::Start()
  {
    //From the Aravis tests/arvexample.c model
    /* Mandatory glib type system initialization */
    arv_g_type_init ();

    if (serialNumber == "")
      {
	/* Instantiation of the first available camera */
	camera = arv_camera_new (NULL);
      }
    else
      {
	//Try to find the given serial number
	camera = arv_camera_new(serialNumber.c_str());
      }

    if (camera == NULL)
      {
	cout << "AravisDriver: No cameras found!" << endl;
	return;
      }
    /* Create a new stream object */
    stream = arv_camera_create_stream (camera, NULL, NULL);
    if (stream == NULL) {
      cout << "AravisDriver: Unable to create stream!" << endl;
      return;
    }
    int payloadSize = arv_camera_get_payload (camera);

    /* Push 50 buffer in the stream input buffer queue */
    for (int i = 0; i < 50; i++)
      arv_stream_push_buffer (stream, arv_buffer_new (payloadSize, NULL));
    
    /* Connect the control-lost signal */
    g_signal_connect (arv_camera_get_device (camera), "control-lost",
		      G_CALLBACK (aravis_driver_control_lost_cb), this);
    
    /* Create a new glib main loop */
    main_loop = g_main_loop_new (NULL, FALSE);
    
    /* Run the main loop */
    g_main_loop_run (main_loop);

    cout << "AravisDriver: Started main loop" << endl;
  }

 
  static void aravis_driver_control_lost_cb (ArvDevice *device, gpointer* data)
  {
    //Since I can't use member functions with G_CALLBACK:
    reinterpret_cast<AravisDriver*>(data)->control_lost_cb(device);
  }
  
  void AravisDriver::control_lost_cb(ArvDevice *device)
  {
    /* Control of the device is lost. Display a message and force application exit */
    cout << "AravisDriver: Control lost" << endl;
  }
  
  void AravisDriver::Stop()
  {
    std::cout << "Stopping Aravis driver" << std::endl;
    g_main_loop_unref (main_loop);
    /* Stop the video stream */
    arv_camera_stop_acquisition (camera);
    g_object_unref (stream);
    
    std::cout << "Stop of Aravis driver complete" << std::endl;
  }


  bool AravisDriver::Capture( hal::CameraMsg& vImages )
  {
    int r;

    vImages.Clear();
    ArvBuffer *buffer;
    buffer = arv_stream_pop_buffer (stream);
    if (buffer == NULL) {
      cout << "AravisDriver: Buffer popped was invalid!" << endl;
      return false;
    }
    ArvBufferPayloadType payloadType = arv_buffer_get_payload_type(buffer);

    if (payloadType != ARV_BUFFER_PAYLOAD_TYPE_IMAGE)
      {
	cout << "AravisDriver: Got unknown payload type: " << payloadType << endl;
	return false;
      }

    
   hal::ImageMsg* pimg = vImages.add_image();
   ArvPixelFormat pixFormat = arv_buffer_get_image_pixel_format(buffer);
   switch (pixFormat)
     {
     case ARV_PIXEL_FORMAT_RGB_8_PACKED:
       pimg->set_type(hal::PB_UNSIGNED_BYTE);
       pimg->set_format(hal::PB_RGB );
       break;
     case ARV_PIXEL_FORMAT_MONO_8:
       pimg->set_type(hal::PB_UNSIGNED_BYTE);
       pimg->set_format(hal::PB_LUMINANCE );
       break;
     default:
       printf( "Unknown pixel format: 0x%x\n", pixFormat);
       return false;
       break;
     }
   pimg->set_width(arv_buffer_get_image_width(buffer));
   pimg->set_height(arv_buffer_get_image_height(buffer));
   
   const void* imageData;
   size_t imageSize;

   imageData = arv_buffer_get_data(buffer, &imageSize);

   pimg->set_data(imageData, imageSize);


    //Return the buffer to the pool to be filled anew
    arv_stream_push_buffer (stream, buffer);
    return true;
  }
  
  /*Split fcn from http://stackoverflow.com/questions/236129/split-a-string-in-c */
  template<typename T>
  vector<T> AravisDriver::split(const T & str, const T & delimiters)
  {
    vector<T> v;
    typename T::size_type start = 0;
    size_t pos = str.find_first_of(delimiters, start);
    while(pos != T::npos) {
      if(pos != start) // ignore empty tokens
	v.push_back(str.substr(start, pos - start));
      start = pos + 1;
      pos = str.find_first_of(delimiters, start);
    }
    if(start < str.length()) // ignore trailing delimiter
      v.push_back(str.substr(start, pos - start)); // add what's left of the string
    return v;
  }
  
  void AravisDriver::reportError(int error_code) {
    fprintf(stderr,"Aravis API Error: %i\n",error_code);
  }
} //namespace
