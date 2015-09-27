#include "OccamDriver.h"



namespace hal {

  OccamDriver::OccamDriver(string m_serial )
    : serialNumber(m_serial)
  {
    
    Start(); //hardcoded for the Occam camera's vid/pid
  }
  
  OccamDriver::~OccamDriver()
  {
    Stop();

  }

  
  size_t OccamDriver::NumChannels() const
  {
    return sensorCount; //one channel for RGB, one for depth
  }

  size_t OccamDriver::Width( size_t idx) const
  {
    return width;
  }

  size_t OccamDriver::Height( size_t idx) const
  {
    return height;
  }

  void OccamDriver::Start()
  {
    int r;
    if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
      {
	reportError(r);
	return;
      }
    
    if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
      {
	reportError(r);
	return;
      }
    
    sensorCount = device_list->entry_count;

    if (sensorCount < 1)
      {
	cout << "OccamDriver: No Occam devices found!" << endl;
	return;
      }
    
    deviceIndex = 0; //default to first one
    
    for (int i = 0; i< device_list->entry_count; i++)
      {
	vector<string> oneCam = split<string>(device_list->entries[i].cid, ":");

	cout << "Found serial number: " << oneCam[1] << endl;

	if (oneCam[1] == serialNumber)
	  {
	    //open the particular one we're interested in
	    deviceIndex = i;
	  }
      }

    cout << "OccamDriver: Opening device: " << device_list->entries[deviceIndex].cid << endl;

    
    if ((r = occamOpenDevice(device_list->entries[deviceIndex].cid, &device)) != OCCAM_API_SUCCESS)
      {
	reportError(r);
	return;
      }
    
    occamGetDeviceValuei(device, OCCAM_SENSOR_WIDTH, &width);
    occamGetDeviceValuei(device, OCCAM_SENSOR_HEIGHT, &height);
    occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensorCount);
    occamGetDeviceValuei(device, OCCAM_COLOR, &isColor);

    cout << "OccamDriver: Found " << sensorCount << " sensors, each [" << width << "x" << height << "]" << endl;
    cout << "OccamDriver: Color? " << isColor << endl;
    
 
  }

  void OccamDriver::Stop()
  {
    std::cout << "Stopping Occam driver" << std::endl;

    occamCloseDevice(device);
    occamFreeDeviceList(device_list);
    occamShutdown();
    
    std::cout << "Stop of Occam driver complete" << std::endl;
  }


  bool OccamDriver::Capture( hal::CameraMsg& vImages )
  {
    int r;

    vImages.Clear();

    //cout << "Woke up!" << endl;
    OccamDataName* req = (OccamDataName*)occamAlloc(sensorCount*sizeof(OccamDataName));
    OccamImage** images = (OccamImage**)occamAlloc(sensorCount*sizeof(OccamImage*));
    for (int i=0;i<sensorCount;i++)
      req[i] = (OccamDataName) (OCCAM_RAW_IMAGE0 + i);

    hal::ImageMsg* pimg;
    if ((r = occamDeviceReadData(device, sensorCount, req, 0, (void**)images, 1)) != OCCAM_API_SUCCESS)
      {
	reportError(r);
	return false;
      }

    for (int i=0; i<sensorCount;i++)
      {
	pimg = vImages.add_image();
    
	if (images[i]->format == OCCAM_GRAY8)
	  {
	    pimg->set_type(hal::PB_UNSIGNED_BYTE);
	    pimg->set_format(hal::PB_LUMINANCE );  
	  }
	else if (images[i]->format == OCCAM_RGB24)
	  {
	    pimg->set_type(hal::PB_UNSIGNED_BYTE);
	    pimg->set_format(hal::PB_RGB );  
	  }
	
          
	pimg->set_width(images[i]->width);
	pimg->set_height(images[i]->height);
 
	pimg->set_data(images[i]->data[0], images[i]->step[0] * images[i]->height);
	occamFreeImage(images[i]);
      }
    
    return true;
  }
  
  /*Split fcn from http://stackoverflow.com/questions/236129/split-a-string-in-c */
  template<typename T>
  vector<T> OccamDriver::split(const T & str, const T & delimiters)
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
  
  void OccamDriver::reportError(int error_code) {
  fprintf(stderr,"Occam API Error: %i\n",error_code);
  }
} //namespace
