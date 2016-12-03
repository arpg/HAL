#include "AravisDriver.h"



namespace hal {

  AravisDriver::AravisDriver(string m_dev,
			     int m_width,
			     int m_height,
			     int m_x,
			     int m_y,
			     int m_exposure)
    : dev(m_dev), exposureTime(m_exposure), roiWidth(m_width), roiHeight(m_height), roiX(m_x), roiY(m_y)
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

  size_t AravisDriver::Width( size_t /* idx */) const
  {
    return width;
  }

  size_t AravisDriver::Height( size_t /* idx */) const
  {
    return height;
  }

  void AravisDriver::Start()
  {
    //From the Aravis tests/arvexample.c model
    /* Mandatory glib type system initialization */
    arv_g_type_init ();

    cout << "Aravis: Init complete!" << endl;

    arv_update_device_list ();

    unsigned int numDevices;
    numDevices = arv_get_n_devices();

    for (unsigned int i=0; i<numDevices; i++)
      {
	printf("Device %s: %s %s %s\n",
	       arv_get_device_id (i),
	       arv_get_device_vendor (i),
	       arv_get_device_model (i),
	       arv_get_device_serial_nbr (i));
      }
 
    if (dev == "")
      {
	/* Instantiation of the first available camera */
	cout << "Aravis: Creating camera" << endl;
	camera = arv_camera_new (NULL);
      }
    else
      {
	//Try to find the given serial number
	printf("Aravis: Looking for camera %s\n", dev.c_str()); 
	camera = arv_camera_new(dev.c_str());
      }
    if (camera == NULL)
      {
	cout << "AravisDriver: No cameras found!" << endl;
	return;
      }

    printf ("Aravis: vendor name           = %s\n", arv_camera_get_vendor_name (camera));
    printf ("Aravis: model name            = %s\n", arv_camera_get_model_name (camera));
    printf ("Aravis: device id             = %s\n", arv_camera_get_device_id (camera));

    //Set the ROI
    arv_camera_get_sensor_size(camera, &width, &height);
    printf ("Aravis: sensor size           =  %ux%u\n", width, height);

    gint reqWidth, reqHeight, reqX, reqY;
    if (roiX + roiWidth > width)
      {
	printf("AravisDriver: Invalid ROI specified, offset: %u + width %u exceeds max: %u\n", roiX, roiWidth, width);
	printf("AravisDriver: Using full width\n");
	reqWidth = width;
	reqX = 0;
      }
    else
      {
	reqWidth = roiWidth;
	reqX = roiX;
      }
    
    if (roiY + roiHeight > height)
      {
	printf("AravisDriver: Invalid ROI specified, offset: %u + height %u exceeds max: %u\n", roiY, roiHeight, height);
	printf("AravisDriver: Using full height\n");
	reqHeight = height;
	reqY = 0;
      }
    else
      {
	reqHeight = roiHeight;
	reqY = roiY;
      }

    if ((reqWidth == 0) && (reqHeight == 0))
      {
	reqWidth = width;
	reqHeight = height;
      }
    else
      {
	width = reqWidth;
	height = reqHeight;
      }
    
    printf("AravisDriver: Setting ROI to: %ux%u+%u+%u\n", reqWidth, reqHeight, reqX, reqY);
 
    
    arv_camera_set_region (camera, reqX, reqY, reqWidth, reqHeight);

    //Set the camera to mono8:
    printf("AravisDriver: Available pixel formats:\n");

    guint numFormats;
    gint64 *pixelFormats = arv_camera_get_available_pixel_formats(camera, &numFormats);

    for (unsigned int i = 0; i< numFormats; i++)
      {
	printf("Format: 0x%x\n", (unsigned int) pixelFormats[i]);
      }
    
    arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8);
    arv_camera_set_exposure_time (camera, exposureTime);
 
    
    
    /* Create a new stream object */
    cout << "Aravis: Creating stream" << endl;
  
    stream = arv_camera_create_stream (camera, NULL, NULL);
    if (stream == NULL) {
      cout << "AravisDriver: Unable to create stream!" << endl;
      return;
    }
    arv_stream_set_emit_signals (stream, TRUE);
    int payloadSize = arv_camera_get_payload (camera);

    printf("Aravis: payload: %d\n", payloadSize);
    
    /* Push 50 buffer in the stream input buffer queue */
    for (int i = 0; i < 50; i++)
      arv_stream_push_buffer (stream, arv_buffer_new (payloadSize, NULL));
    
    /* Connect the control-lost signal */
    g_signal_connect (arv_camera_get_device (camera), "control-lost",
		      G_CALLBACK (aravis_driver_control_lost_cb), this);
    g_signal_connect (stream, "new-buffer", G_CALLBACK (aravis_driver_new_buffer_cb), this);

    
    //Start the camera doing its thing
    arv_camera_set_acquisition_mode (camera, ARV_ACQUISITION_MODE_CONTINUOUS);
    arv_camera_start_acquisition (camera);
    capStart = hal::Tic();
    
    /* Create a new glib main loop */
    main_loop = g_main_loop_new (NULL, FALSE);

    loopThread = new std::thread(&AravisDriver::threadFunc, this);
  }

 
  static void aravis_driver_control_lost_cb (ArvDevice *device, gpointer* data)
  {
    //Since I can't use member functions with G_CALLBACK:
    reinterpret_cast<AravisDriver*>(data)->control_lost_cb(device);
  }
  
  void AravisDriver::control_lost_cb(ArvDevice* /* device */)
  {
    /* Control of the device is lost. Display a message and force application exit */
    cout << "AravisDriver: Control lost" << endl;
  }
  
  static void aravis_driver_new_buffer_cb (ArvStream *stream, gpointer* data)
  {
    //Since I can't use member functions with G_CALLBACK:
    reinterpret_cast<AravisDriver*>(data)->new_buffer_cb(stream);
  }
  
  void AravisDriver::new_buffer_cb(ArvStream* m_stream)
  {
    std::lock_guard<std::mutex> guard(bufferMutex);
    buffer = arv_stream_pop_buffer (m_stream);
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
  
  cv::Mat* AravisDriver::unBayer(ArvBuffer *buffer)
  {

   const void* imageData;
   size_t origSize;
   imageData = arv_buffer_get_data(buffer, &origSize);
   cv::Mat src(arv_buffer_get_image_height(buffer),
	       arv_buffer_get_image_width(buffer),
	       CV_8UC1,
	       (void*) imageData);
   
   //Create it on the heap
   cv::Mat *dst = new cv::Mat(arv_buffer_get_image_height(buffer),
	       arv_buffer_get_image_width(buffer),
	       CV_8UC3);
   cv::cvtColor(src, *dst, CV_BayerGB2RGB);
   
   return dst;
	       
  }
  
  bool AravisDriver::Capture( hal::CameraMsg& vImages )
  {
    capStart = hal::Tic();
    vImages.Clear();

    //Lock the buffer 
    std::lock_guard<std::mutex> guard(bufferMutex);

    if (buffer == NULL) {
      cout << "AravisDriver: Buffer popped was invalid!" << endl;
      //arv_stream_push_buffer (stream, buffer);
      return false;
    }
    if (arv_buffer_get_status (buffer) != ARV_BUFFER_STATUS_SUCCESS)
      {
	//cout << "AravisDriver: Buffer was not successful :(" << endl;
	arv_stream_push_buffer (stream, buffer);
	return false;
      }
    ArvBufferPayloadType payloadType = arv_buffer_get_payload_type(buffer);

    if (payloadType != ARV_BUFFER_PAYLOAD_TYPE_IMAGE)
      {
	cout << "AravisDriver: Got unknown payload type: " << payloadType << endl;
	arv_stream_push_buffer (stream, buffer);
	return false;
      }

    hal::ImageMsg* pimg = vImages.add_image();
   
    const void* imageData;
    size_t imageSize;
    pimg->set_width(arv_buffer_get_image_width(buffer));
    pimg->set_height(arv_buffer_get_image_height(buffer));
   
    imageData = arv_buffer_get_data(buffer, &imageSize);
    ArvPixelFormat pixFormat = arv_buffer_get_image_pixel_format(buffer);

    cv::Mat* bayerImage;

    switch (pixFormat)
      {
      case ARV_PIXEL_FORMAT_BAYER_GB_8:
	//Need to unbayer via OpenCV first
	pimg->set_type(hal::PB_UNSIGNED_BYTE);
	pimg->set_format(hal::PB_BGR );
       
	bayerImage = unBayer(buffer);

	imageSize =bayerImage->total() * bayerImage->elemSize();
	//printf("Debayer time: %1.6f\n", hal::Toc(capStart));
       
	pimg->set_data(static_cast<const unsigned char*>(bayerImage->data), imageSize);
	//printf("Image message time: %1.6f\n", hal::Toc(capStart));

	//cv::imshow("Debayered", *bayerImage);
	//cv::waitKey(0);
	delete bayerImage;
       
	break;
      case ARV_PIXEL_FORMAT_MONO_8:
	pimg->set_type(hal::PB_UNSIGNED_BYTE);
	pimg->set_format(hal::PB_LUMINANCE );

	//Image data usable directly
	pimg->set_data(imageData, imageSize);
	 
	break;
      default:
	printf( "Unknown pixel format: 0x%x\n", pixFormat);
	arv_stream_push_buffer (stream, buffer);
	return false;
	break;
      }

    //Return the buffer to the pool to be filled anew
    arv_stream_push_buffer (stream, buffer);

    vImages.set_system_time(hal::Tic());
    //printf("Total capture time: %1.6f\n", hal::Toc(capStart));
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

  void AravisDriver::threadFunc()
  {
    cout << "Aravis: Starting main loop" << endl;
    /* Run the main loop */
    g_main_loop_run (main_loop);
  }
} //namespace
