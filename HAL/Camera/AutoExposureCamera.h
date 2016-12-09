#pragma once

#include <HAL/Camera/CameraDriverInterface.h>

namespace hal
{

class AutoExposureCamera : public CameraDriverInterface
{
  public:

    virtual int GetAutoExposureImageIndex() = 0;

    virtual void DisableAutoExposure() = 0;

    virtual double GetActualFramerate() = 0;

    virtual double GetDesiredFramerate() = 0;

    virtual double GetExposure() = 0;

    virtual void SetExposure(double exposure) = 0;

    virtual double GetGain() = 0;

    virtual void SetGain(double gain) = 0;

    virtual double GetMaxGain() = 0;

    virtual double GetMinGain() = 0;
};

} // namespace hal