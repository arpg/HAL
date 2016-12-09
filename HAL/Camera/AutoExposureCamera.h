#pragma once

#include <HAL/Camera/CameraDriverInterface.h>

namespace hal
{

class AutoExposureCamera : public CameraDriverInterface
{
  public:

    virtual int GetAutoExposureImageIndex() const = 0;

    virtual void DisableAutoExposure() = 0;

    virtual double GetActualFramerate() const = 0;

    virtual double GetDesiredFramerate() const = 0;

    virtual double GetExposure() const = 0;

    virtual void SetExposure(double exposure) = 0;

    virtual double GetGain() const = 0;

    virtual void SetGain(double gain) = 0;

    virtual double GetMaxGain() const = 0;

    virtual double GetMinGain() const = 0;
};

} // namespace hal