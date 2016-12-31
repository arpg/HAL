#include "AravisDriver.h"

#define MAX_BAD_BUFFERS 10

namespace hal {

  AravisDriver::AravisDriver(string m_dev,
			     int m_width,
			     int m_height,
			     int m_x,
			     int m_y,
			     int m_exposure,
			     float m_gain,
			     int m_bandwidth,
			     int m_sync,
			     string m_syncPort,
			     string m_syncSource)
    : dev(m_dev), roiWidth(m_width), roiHeight(m_height), roiX(m_x), roiY(m_y),
      exposureTime(m_exposure), gain(m_gain), bandwidthLimit(m_bandwidth),
      sync(m_sync), syncPort(m_syncPort), syncSource(m_syncSource)
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

    //Make sure we're not acquiring currently
    arv_camera_stop_acquisition(camera);
    
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

    // The camera has some interesting limitations on the payload, with the width and height
    // Width and height have to be multiples of 4
    // Payload has to be divisible by 0x800

    uint32_t payloadSize = reqWidth*reqHeight;
    if ((reqWidth % 4 != 0) || (reqHeight % 4 != 0) || (payloadSize % 0x800 != 0))
      {
	printf("AravisDriver: Your requested ROI may not work: \n");
	printf("AravisDriver: width %% 4: %u\n", reqWidth % 4);
	printf("AravisDriver: height %% 4: %u\n", reqHeight % 4);
	printf("AravisDriver: payloadSize of %u %% 0x800: %u\n", payloadSize, payloadSize % 0x800);
	printf("AravisDriver: Check to see if the camera accepts your settings\n");
      }
    
    printf("AravisDriver: Setting ROI to: %ux%u+%u+%u\n", reqWidth, reqHeight, reqX, reqY);
 
    
    arv_camera_set_region (camera, reqX, reqY, reqWidth, reqHeight);
    arv_camera_set_region (camera, reqX, reqY, reqWidth, reqHeight);
    
    arv_camera_get_region (camera, &reqX, &reqY, &reqWidth, &reqHeight);
    printf("AravisDriver: Camera reports ROI: %ux%u+%u+%u\n", reqWidth, reqHeight, reqX, reqY);
 
    
    printf("AravisDriver: Available pixel formats:\n");

    guint numFormats;
    const char **pixelFormats = arv_camera_get_available_pixel_formats_as_strings(camera, &numFormats);

    for (unsigned int i = 0; i< numFormats; i++)
      {
	printf("Format: %s\n", pixelFormats[i]);
      }
    g_free(pixelFormats);
    
    //Set the camera to mono8:
    //arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8);

    printf("AravisDriver: Setting exposure to %1.2f\n", exposureTime);
    arv_camera_set_exposure_time (camera, exposureTime);
 
    exposureTime = arv_camera_get_exposure_time (camera);
    printf("AravisDriver: Camera reports exposure: %1.2f\n", exposureTime);
    
    double minGain, maxGain;
    arv_camera_get_gain_bounds(camera, &minGain, &maxGain);
    
    printf("AravisDriver: Setting gain to %1.2f, limits: %1.2f to %1.2f\n", gain, minGain, maxGain);
    arv_camera_set_gain(camera, gain);
    gain = arv_camera_get_gain(camera);
    printf("AravisDriver: Camera reports gain of %1.2f\n", gain);

    //Set the bandwidth limit if specified
    if (bandwidthLimit > 0)
      {
	//Check for Aravis support:
	if (arv_camera_uv_is_bandwidth_control_available(camera))
	  {
	    unsigned int min,max;
	    unsigned int bandwidthLimitu = (unsigned int) bandwidthLimit;
	    arv_camera_uv_get_bandwidth_bounds(camera, &min, &max);
	    min /= 1e6;
	    max /= 1e6;
	    //Limits are internally represented in bytes/s - somewhat cumbersome
	    printf("AravisDriver: Bandwidth limits are: (%d,%d)\n", min, max); 
	    if ((bandwidthLimitu > min) && (bandwidthLimitu < max))
	      {
		printf("AravisDriver: Set bandwidth limit to: %d\n", bandwidthLimit);
		arv_camera_uv_set_bandwidth(camera, bandwidthLimitu*1e6);
	      }

	  }
	else
	  {
	    printf("AravisDriver: Bandwidth control is not available on this camera\n");
	    
	  }
      }

    /* Advertise the available sync settings */
    const char** gpioLines;
    guint numLines;
    gpioLines = arv_camera_get_gpio_lines(camera, &numLines);

    for (unsigned int i=0; i<numLines; i++)
      {
	printf("AravisDriver: Found GPIO output line: %s\n", gpioLines[i]);
      }
    const char** gpioSources;
    guint numSources;
	
    gpioSources = arv_camera_get_gpio_output_sources(camera, &numSources);
    for (unsigned int i=0; i<numSources; i++)
      {
	printf("AravisDriver: Found GPIO output source: %s\n", gpioSources[i]);
      }
    const char** triggerSources;
    guint numTriggers;
    triggerSources = arv_camera_get_trigger_sources(camera, &numTriggers);
    for (unsigned int i=0; i < numTriggers; i++)
      {
	printf("AravisDriver: Found sync trigger source: %s\n", triggerSources[i]);
      }
    
    /* Configure the sync settings */
    switch (sync)
      {
      case 0: //Freerun:
	printf("AravisDriver: Camera is in freerun mode\n");
	arv_camera_clear_triggers(camera);
	break;
      case 1: //Slave sync
	if (syncPort == "")
	  syncPort = "Line1";
	
	arv_camera_set_trigger (camera, syncPort.c_str());
	printf("AravisDriver: Camera syncing to %s\n", syncPort.c_str());
	break;
      case 2: //Master sync
	if (syncPort == "")
	  syncPort = "Line2";
	printf("AravisDriver: Setting output line: %s as master\n", syncPort.c_str());
	arv_camera_set_gpio_mode(camera, syncPort.c_str(), ARV_GPIO_MODE_OUTPUT);

	ArvGpioMode syncState = arv_camera_get_gpio_mode(camera, syncPort.c_str());
	if (syncState != ARV_GPIO_MODE_OUTPUT)
	  {
	    printf("AravisDriver: The camera doesn't like setting that port for output - master sync not enabled\n");
	  }
	else
	  {
	    printf("AravisDriver: Setting GPIO output source to: %s\n", syncSource.c_str());
	    arv_camera_set_gpio_output_source(camera, syncPort.c_str(), syncSource.c_str());

	    //Output sync also requires software triggering for some unclear reason
	    arv_camera_set_trigger(camera, "Software");
	  }
	
      }
    
    /* Create a new stream object */
    cout << "Aravis: Creating stream" << endl;
  
    stream = arv_camera_create_stream (camera, NULL, NULL);
    if (stream == NULL) {
      cout << "AravisDriver: Unable to create stream!" << endl;
      return;
    }
    arv_stream_set_emit_signals (stream, TRUE);
    payloadSize = arv_camera_get_payload (camera);

    printf("Aravis: payload: %d\n", payloadSize);
    
    /* Push 50 buffer in the stream input buffer queue */
    for (int i = 0; i < 50; i++)
      arv_stream_push_buffer (stream, arv_buffer_new (payloadSize, NULL));

    sharedBuffer = NULL;
    /* Connect the control-lost signal */
    g_signal_connect (arv_camera_get_device (camera), "control-lost",
		      G_CALLBACK (aravis_driver_control_lost_cb), this);
    g_signal_connect (stream, "new-buffer", G_CALLBACK (aravis_driver_new_buffer_cb), this);

    badBufferCount = 0;
    
    //Start the camera doing its thing
    arv_camera_set_acquisition_mode (camera, ARV_ACQUISITION_MODE_CONTINUOUS);
    arv_camera_start_acquisition (camera);
    arv_camera_stop_acquisition (camera);
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
    std::unique_lock<std::mutex> guard(bufferMutex);
    if (sharedBuffer != NULL)
      {
	//Push the existing buffer back
	arv_stream_push_buffer (m_stream, sharedBuffer);
      }
    
    //Freshen the working buffer	
    sharedBuffer = arv_stream_pop_buffer (m_stream);
    guard.unlock();
    bufferBell.notify_one();
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
    std::unique_lock<std::mutex> lock(bufferMutex);
    while (badBufferCount < MAX_BAD_BUFFERS)
      {
	if (sync == 2)
	    arv_camera_software_trigger(camera);

	if (bufferBell.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::timeout)
	  {
	    printf("AravisDriver: Timed out waiting for an image!\n");;
	    badBufferCount++;
	    continue;
	  }
	
	if (sharedBuffer == NULL) {
	   printf("AravisDriver: Buffer popped was invalid!\n");
	   badBufferCount++;
	   continue;
	}
	if (arv_buffer_get_status (sharedBuffer) == ARV_BUFFER_STATUS_SIZE_MISMATCH)
	  {
	    printf("AravisDriver: Buffer was underrun, attempt: %d\n", badBufferCount);;
	    
	    badBufferCount++;
	    continue;
	  }

	if (arv_buffer_get_status(sharedBuffer) == ARV_BUFFER_STATUS_SUCCESS)
	  break;
      }
    badBufferCount = 0;
        
    ArvBufferPayloadType payloadType = arv_buffer_get_payload_type(sharedBuffer);

    if (payloadType != ARV_BUFFER_PAYLOAD_TYPE_IMAGE)
      {
	cout << "AravisDriver: Got unknown payload type: " << payloadType << endl;
	return false;
      }

    hal::ImageMsg* pimg = vImages.add_image();
   
    const void* imageData;
    size_t imageSize;
    pimg->set_width(arv_buffer_get_image_width(sharedBuffer));
    pimg->set_height(arv_buffer_get_image_height(sharedBuffer));
   
    imageData = arv_buffer_get_data(sharedBuffer, &imageSize);
    ArvPixelFormat pixFormat = arv_buffer_get_image_pixel_format(sharedBuffer);

    cv::Mat* bayerImage;

    switch (pixFormat)
      {
      case ARV_PIXEL_FORMAT_BAYER_GB_8:
	//Need to unbayer via OpenCV first
	pimg->set_type(hal::PB_UNSIGNED_BYTE);
	pimg->set_format(hal::PB_BGR );
       
	bayerImage = unBayer(sharedBuffer);

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
	printf( "AravisDriver: Unknown pixel format: 0x%x\n", pixFormat);
	return false;
	break;
      }


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
