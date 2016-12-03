#pragma once

#include <memory>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Devices/DeviceException.h>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include <unistd.h>
#include <pthread.h>

//Protobuf includes
#include <HAL/Image.pb.h>
#include <HAL/Messages/ImageArray.h>
#include <HAL/Utils/TicToc.h>

//For debayering
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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
  static void aravis_driver_control_lost_cb(ArvDevice *device, gpointer *data);
  static void aravis_driver_new_buffer_cb(ArvStream *stream, gpointer *data);
  
  class AravisDriver : public CameraDriverInterface
  {
  public:
    AravisDriver(string m_serial,
		 int m_width,
		 int m_height,
		 int m_x,
		 int m_y,
		 int m_exposure);
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
    cv::Mat* unBayer(ArvBuffer *buffer);
    
    void control_lost_cb(ArvDevice *device);
    void new_buffer_cb(ArvStream *stream);
    void threadFunc();
    
    string dev;
    int sensorCount;
    int deviceIndex;
    int isColor;
    bool cancel;
    //Width/Height of each component image
    int width;
    int height;
    int x;
    int y;

    double exposureTime; //in microseconds
    int roiWidth;
    int roiHeight;
    int roiX;
    int roiY;
    
    double capStart;
    
    //Aravis variables
    ArvCamera *camera;
    ArvStream *stream;
    ArvBuffer *buffer;
    std::mutex bufferMutex;
    
    GMainLoop *main_loop; //for GLib signals
    std::thread *loopThread; //to run the GLib event loop
    
  };

}
