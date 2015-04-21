#include "ROSDriver.h"



namespace hal {

  ROSDriver::ROSDriver(string m_topics, int width, int height )
    : topics(m_topics)
  {
    int argc = 0;
    ros::init(argc, NULL, "ros_hal");
    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    nh = new ros::NodeHandle("~");
    it = new image_transport::ImageTransport(*nh);
    topicBell = PTHREAD_MUTEX_INITIALIZER;
    width_ = width;
    height_ = height;
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

  size_t ROSDriver::NumChannels() const
  {
    return topicCount; //one channel for RGB, one for depth
  }

  size_t ROSDriver::Width( size_t /*idx*/) const
  {
    return width_;
  }

  size_t ROSDriver::Height( size_t /*idx*/) const
  {
    return height_;
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


  bool ROSDriver::Capture( pb::CameraMsg& vImages )
  {
    //Only publish when there's at least one new image to share

    pthread_cond_wait(&newTopic, &topicBell);
    vImages.Clear();

    //cout << "Woke up!" << endl;
    pb::ImageMsg* pimg;

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
	pimg->set_type(findPbType(freshImages[i]->encoding));
	pimg->set_format(findPbFormat(freshImages[i]->encoding) );            
	pimg->set_width(freshImages[i]->width);
	pimg->set_height(freshImages[i]->height);
	pimg->set_data(&freshImages[i]->data[0], freshImages[i]->step * freshImages[i]->height);
      }
    
    pthread_mutex_unlock(&topicBell);
    return true;
  }
  string ROSDriver::findROSFormat(uint32_t format)
  {
    switch (format)
      {
      case pb::PB_LUMINANCE:
	return sensor_msgs::image_encodings::MONO8;
      case pb::PB_RGB:
	return sensor_msgs::image_encodings::RGB8;
      case pb::PB_RGBA:
	return sensor_msgs::image_encodings::RGBA8;
      case pb::PB_RAW:
	return sensor_msgs::image_encodings::MONO8;
      case pb::PB_BGR:
	return sensor_msgs::image_encodings::BGR8;
      case pb::PB_BGRA:
	return sensor_msgs::image_encodings::BGRA8;
      default:
	ROS_FATAL("Unknown HAL image format: 0x%x\n", format);
	return NULL;
      }
  }

  pb::Type ROSDriver::findPbType(string format)
  {
    if (format == sensor_msgs::image_encodings::MONO8)
      return pb::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::BGR8)
      return pb::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::RGB8)
      return pb::PB_UNSIGNED_BYTE;
    if (format == sensor_msgs::image_encodings::MONO16)
      return pb::PB_UNSIGNED_SHORT;
    if (format == sensor_msgs::image_encodings::TYPE_16UC1)
      return pb::PB_UNSIGNED_SHORT;
    ROS_FATAL("Unknown ROS image format: [%s]\n", format.c_str());
    return (pb::Type) 0;
  }
  
  pb::Format ROSDriver::findPbFormat(string format)
  {
    //Protobuf formats are typedefed ints, ROS formats are strings
    if (format == sensor_msgs::image_encodings::MONO8)
      return pb::PB_LUMINANCE;
  
    if (format == sensor_msgs::image_encodings::MONO16)
      return pb::PB_LUMINANCE;
    
    if (format == sensor_msgs::image_encodings::TYPE_16UC1)
      return pb::PB_LUMINANCE;

    if (format == sensor_msgs::image_encodings::BGR8)
      return pb::PB_BGR;
  
    if (format == sensor_msgs::image_encodings::RGB8)
      return pb::PB_RGB;
  
    ROS_FATAL("Unknown ROS image format: [%s]\n", format.c_str());
    return (pb::Format) 0;
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

  
} //namespace
