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

//Aravis includes for GLib, etc - from the tests/arvexample.c source

#include <arv.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>

using std::cout;
using std::string;
using std::endl;
using std::vector;


namespace hal
{

  class AravisDriver : public CameraDriverInterface
  {
  public:
    AravisDriver(string m_serial);
    ~AravisDriver();

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }
 
    void Start();
    void Stop();
    
    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;
    template<typename T> vector<T> split(const T & str, const T & delimiters);
    void reportError(int error_code);
    void control_lost_cb(ArvDevice *device);
    
    string serialNumber;
    int sensorCount;
    int deviceIndex;
    int isColor;
    bool cancel;
    //Width/Height of each component image
    int width;
    int height;
    
    //Aravis variables
    ArvCamera *camera;
    ArvStream *stream;
    GMainLoop *main_loop; //for GLib signals
  };

}
