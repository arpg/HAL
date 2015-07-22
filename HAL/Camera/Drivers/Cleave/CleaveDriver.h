#pragma once

#include <memory>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <pthread.h>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include <HAL/Devices/DeviceException.h>

//Protobuf includes
#include <HAL/Image.pb.h>
#include <HAL/Messages/ImageArray.h>



namespace hal
{

  class CleaveDriver : public CameraDriverInterface
  {
  public:
    CleaveDriver(std::shared_ptr<CameraDriverInterface> Input, int m_maxChannel, int m_minChannel);
    ~CleaveDriver();

    bool Capture( hal::CameraMsg& vImages );
    std::shared_ptr<CameraDriverInterface> GetInputDevice() { return inputCamera; }

    void Start();
    void Stop();

    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;


    //Width/Height of each component image
    std::vector<size_t> widths;
    std::vector<size_t> heights;
    unsigned int maxChannel;
    unsigned int minChannel;

    std::shared_ptr<CameraDriverInterface>  inputCamera;
    hal::CameraMsg                           m_InMsg;

  };

}
