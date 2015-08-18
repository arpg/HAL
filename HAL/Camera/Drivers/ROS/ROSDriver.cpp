#include "ROSDriver.h"



namespace hal {

  ROSDriver::ROSDriver(string m_topics, string m_sizes, int m_grayScale )
    : topics(m_topics), sizes(m_sizes), grayScale(m_grayScale)
  {
    int argc = 0;
    ros::init(argc, NULL, "ros_hal", ros::init_options::AnonymousName );
    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    nh = new ros::NodeHandle("~");
    it = new image_transport::ImageTransport(*nh);
    topicBell = PTHREAD_MUTEX_INITIALIZER;

    //Parse the sizes string so that we have that available when called by higher
    parseSizes();
    Start(); //hardcoded for the ROS camera's vid/pid
  }
  
  ROSDriver::~ROSDriver()
  {
    Stop();

    delete[] subscribers;
    delete[] freshImages;
    delete[] topicLocks;
    delete it;
    delete nh;
  }
  
  void ROSDriver::parseSizes()
  {
    vector<string> sizeList = split<string>(sizes, "+");
    int numSizes = sizeList.size();
    for (int i=0; i< numSizes; i++)
      {
	vector<string> oneSize = split<string>(sizeList[i], "x");
	widthList.push_back(strtoul(oneSize[0].c_str(), NULL, 10));
	heightList.push_back(strtoul(oneSize[1].c_str(), NULL, 10));
      }
  }
  
  size_t ROSDriver::NumChannels() const
  {
    return topicCount; //one channel for RGB, one for depth
  }

  size_t ROSDriver::Width( size_t idx) const
  {
    return widthList[idx];
  }

  size_t ROSDriver::Height( size_t idx) const
  {
    return heightList[idx];
  }

  void ROSDriver::Start()
  {
    
    //Split the topic list and instantiate a subscriber for each one
    topicList = split<string>(topics, "+");
    topicCount = topicList.size();

    cout << "Creating " << topicCount << " subscribers" << endl;
    
    topicLocks = new pthread_mutex_t[topicCount];
    freshImages = new sensor_msgs::ImageConstPtr[topicCount];
   
    pthread_cond_init(&newTopic, NULL);
    initComplete = 0; //no images yet
    
    subscribers = new image_transport::Subscriber[topicCount];
    for (int i=0; i< topicCount; i++)
      {
	cout << "Subscribing to: " << topicList[i] << endl;
	pthread_mutex_init(&topicLocks[i], NULL);
	freshImages[i] = NULL;
	subscribers[i] = it->subscribe(topicList[i], 1000, boost::bind(&ROSDriver::imageCallback, this, _1,i));
     }


  }

  void ROSDriver::Stop()
  {
    std::cout << "Stopping ROS driver" << std::endl;
    std::cout << "Stop of ROS driver complete" << std::endl;
  }


  bool ROSDriver::Capture( hal::CameraMsg& vImages )
  {
    //Only publish when there's at least one new image to share
    //Return false if we timed out in the cond_wait
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec +=1;

    int ret;
    
    ret = pthread_cond_timedwait(&newTopic, &topicBell, &timeout);
    if (ret == ETIMEDOUT)
      {
	std::cout << "No images received from ROS!" << std::endl;
	return false;
      }

    vImages.Clear();

    //cout << "Woke up!" << endl;
    hal::ImageMsg* pimg;

    if (!initComplete)
      {
	//First run only:
	//release the mutex, let the callback populate some buffers for us
	//basically, this pauses to let imagery accumulate on the channels
	pthread_mutex_unlock(&topicBell);
	int imagesValid = 0;
	while (!imagesValid)
	  {
	    pthread_mutex_unlock(&topicBell);
	    usleep(1000);
	    imagesValid = 1;
	    pthread_mutex_lock(&topicBell);
	    for (int i=0; i < topicCount; i++)
	      {
		if (freshImages[i] == NULL)
		  {
		    imagesValid = 0;
		  }
	      }
	   
	  }
	initComplete = 1;
      }
    
    for (int i =0; i< topicCount; i++)
      {
	pimg = vImages.add_image();


	//Pass through the timestamp
	double halStamp = freshImages[i]->header.stamp.sec + ((double)freshImages[i]->header.stamp.nsec)/1e9;
	
	pimg->set_timestamp(halStamp);
	pimg->set_type(findPbType(freshImages[i]->encoding));
	pimg->set_format(findPbFormat(freshImages[i]->encoding) );            
	pimg->set_width(freshImages[i]->width);
	pimg->set_height(freshImages[i]->height);

	//If necessary, convert the floating point images types to fixed point
	if (freshImages[i]->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
	  {
	    sensor_msgs::ImagePtr convImage;
	    makeFixedPoint(convImage, freshImages[i]);
	    pimg->set_data(&convImage->data[0], convImage->step * convImage->height);
	  }
	else if ((freshImages[i]->encoding == sensor_msgs::image_encodings::MONO16 ||
		  freshImages[i]->encoding == sensor_msgs::image_encodings::TYPE_16UC1) &&
		 grayScale)
	  {
	    
	    //Dynamically adjust the scale of the uint16_t to occupy the full range
	    //Certain cameras only publish 10 or 12-bit data, leading to very dark images
	    //First, go through the image and find the real max value
	    //Then, rescale the image into the full 16-bit range
	    sensor_msgs::ImagePtr scaledImage;
	    makeScaledImage(scaledImage, freshImages[i]);
	    pimg->set_type(hal::PB_UNSIGNED_BYTE);
	    pimg->set_data(&scaledImage->data[0], scaledImage->step * scaledImage->height);
 
	  }
	else
	  {
	    //the image published from ROS is already a uint* type of image
	    pimg->set_data(&freshImages[i]->data[0], freshImages[i]->step * freshImages[i]->height);
	  }
      }
    
    pthread_mutex_unlock(&topicBell);
    return true;
  }
  string ROSDriver::findROSFormat(uint32_t format)
  {
    switch (format)
      {
      case hal::PB_LUMINANCE:
	return sensor_msgs::image_encodings::MONO8;
      case hal::PB_RGB:
	return sensor_msgs::image_encodings::RGB8;
      case hal::PB_RGBA:
	return sensor_msgs::image_encodings::RGBA8;
      case hal::PB_RAW:
	return sensor_msgs::image_encodings::MONO8;
      case hal::PB_BGR:
	return sensor_msgs::image_encodings::BGR8;
      case hal::PB_BGRA:
	return sensor_msgs::image_encodings::BGRA8;
      default:
	ROS_FATAL("Unknown HAL image format: 0x%x\n", format);
	return NULL;
      }
  }

  hal::Type ROSDriver::findPbType(string format)
  {
    if (format == sensor_msgs::image_encodings::MONO8)
      return hal::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::TYPE_8UC1)
      return hal::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::BGR8)
      return hal::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::RGB8)
      return hal::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::MONO16)
      return hal::PB_UNSIGNED_SHORT;
    if (format == sensor_msgs::image_encodings::TYPE_8UC1)
      return hal::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::TYPE_16UC1)
      return hal::PB_UNSIGNED_SHORT;
    if (format == sensor_msgs::image_encodings::TYPE_32FC1)
      return hal::PB_UNSIGNED_SHORT;
    ROS_FATAL("Unknown ROS image format: [%s]\n", format.c_str());
    return (hal::Type) 0;
  }
  
  hal::Format ROSDriver::findPbFormat(string format)
  {
    //Protobuf formats are typedefed ints, ROS formats are strings
    if (format == sensor_msgs::image_encodings::MONO8)
      return hal::PB_LUMINANCE;
    
    if (format == sensor_msgs::image_encodings::TYPE_8UC1)
      return hal::PB_LUMINANCE;

    if (format == sensor_msgs::image_encodings::MONO16)
      return hal::PB_LUMINANCE;
    
    if (format == sensor_msgs::image_encodings::TYPE_8UC1)
      return hal::PB_LUMINANCE;

    if (format == sensor_msgs::image_encodings::TYPE_16UC1)
      return hal::PB_LUMINANCE;
    
    if (format == sensor_msgs::image_encodings::TYPE_32FC1)
      return hal::PB_LUMINANCE;

    if (format == sensor_msgs::image_encodings::BGR8)
      return hal::PB_BGR;
  
    if (format == sensor_msgs::image_encodings::RGB8)
      return hal::PB_RGB;
  
    ROS_FATAL("Unknown ROS image format: [%s]\n", format.c_str());
    return (hal::Format) 0;
  }

  
  /*Split fcn from http://stackoverflow.com/questions/236129/split-a-string-in-c */
  template<typename T>
  vector<T> ROSDriver::split(const T & str, const T & delimiters)
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

  void ROSDriver::imageCallback(const sensor_msgs::Image::ConstPtr& msg, int topicIndex)
  {
    //Something arrived - put it in the right spot!
    //cout << "Updating image: " << topicIndex << endl;
    pthread_mutex_lock(&topicLocks[topicIndex]);
    freshImages[topicIndex] = msg;

    pthread_mutex_unlock(&topicLocks[topicIndex]);
    //Ring the bell
    pthread_cond_signal(&newTopic);
  }

  void ROSDriver::makeFixedPoint(sensor_msgs::ImagePtr &destImage_sp, const sensor_msgs::ImageConstPtr srcImage)
  {
    //Given a src floating point grayscale image, convert to a fixed radix point grayscale image
    //Radix point as class const for now
     sensor_msgs::Image* destImage = new sensor_msgs::Image;
    destImage->encoding = sensor_msgs::image_encodings::MONO16;
    destImage->height = srcImage->height;
    destImage->width = srcImage->width;
    destImage->step = srcImage->width*sizeof(uint16_t);
    destImage->data.resize(destImage->step * destImage->height);

    unsigned int i;

    float *baseSrc = (float*)(&srcImage->data[0]);
    uint16_t *baseDest = (uint16_t*) (&destImage->data[0]); //since the data ptr is a uint8_t for both storage media
    
    for (i = 0; i< srcImage->height*srcImage->width; i++)
      {
	baseDest[i] = baseSrc[i] * radixMultiplier; //cut off at the uint16_t boundary
      }

    destImage_sp = boost::shared_ptr<sensor_msgs::Image>(destImage);
  }

  void ROSDriver::makeScaledImage(sensor_msgs::ImagePtr &destImage_sp, const sensor_msgs::ImageConstPtr srcImage)
  {
    //Dynamically scale a gray image to span the full uint16_t space
    //since some cameras only publish 10 or 12-bit data
    
     sensor_msgs::Image* destImage = new sensor_msgs::Image;
    destImage->encoding = sensor_msgs::image_encodings::MONO8;
    destImage->height = srcImage->height;
    destImage->width = srcImage->width;
    destImage->step = srcImage->width*sizeof(uint8_t);
    destImage->data.resize(destImage->step * destImage->height);

    unsigned int i;

    uint16_t *baseSrc = (uint16_t*)(&srcImage->data[0]);
    uint8_t *baseDest = (uint8_t*) (&destImage->data[0]); //since the data ptr is a uint8_t for both storage media
    //use two passes: one to find this images' max
    //second to actually scale every value

    uint16_t maxValue = 0;
    uint8_t newValue = 0;
    
    for (i = 0; i< srcImage->height*srcImage->width; i++)
      {
	if (baseSrc[i] > maxValue)
	  maxValue = baseSrc[i];
      }

    //ROS_INFO("Found max value: %u\n", maxValue);
    //Scale the image
    for (i = 0; i< srcImage->height*srcImage->width; i++)
      {
	newValue = (float) baseSrc[i] / maxValue * (1 << 8);
	baseDest[i] =  newValue;
      }

    
    destImage_sp = boost::shared_ptr<sensor_msgs::Image>(destImage);
  }
	   
  
} //namespace
