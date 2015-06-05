#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Devices/DeviceException.h>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>

#include <unistd.h>
#include <pthread.h>

//Protobuf includes
#include <HAL/Image.pb.h>
#include <HAL/Messages/ImageArray.h>

//Occam includes from the indigosdk package
#include <occam/indigo.h>

using std::cout;
using std::string;
using std::endl;
using std::vector;


namespace hal
{

  class OccamDriver : public CameraDriverInterface
  {
  public:
    OccamDriver(string m_serial);
    ~OccamDriver();

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }
 
    void Start();
    void Stop();
    
    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;
    template<typename T> vector<T> split(const T & str, const T & delimiters);
    void reportError(int error_code);
    
    string serialNumber;
    int sensorCount;
    int deviceIndex;
    int isColor;
    
    //Width/Height of each component image
    int width;
    int height;
    
    //Occam variables
    OccamDeviceList* device_list;
    OccamDevice* device;
    OccamImage* image;
    
  };

}
