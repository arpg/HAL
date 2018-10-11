#pragma once

#include <HAL/Camera/CameraDriverInterface.h>

namespace hal
{

class AutoExposureInterface : public CameraDriverInterface
{
  public:

    virtual double MaxExposure(int channel = 0) const = 0;

    virtual double MinExposure(int channel = 0) const = 0;

    virtual double MaxGain(int channel = 0) const = 0;

    virtual double MinGain(int channel = 0) const = 0;

    virtual double Exposure(int channel = 0) = 0;

    virtual void SetExposure(double exposure, int channel = 0) = 0;

    virtual double Gain(int channel = 0) = 0;

    virtual void SetGain(double gain, int channel = 0) = 0;

    virtual double ProportionalGain(int channel = 0) const = 0;

    virtual double IntegralGain(int channel = 0) const = 0;

    virtual double DerivativeGain(int channel = 0) const = 0;
};

} // namespace hal