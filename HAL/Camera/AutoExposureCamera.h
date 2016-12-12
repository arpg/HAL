#pragma once

#include <cfloat>
#include <cmath>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Devices/DeviceException.h>

#include <iostream> // TODO: remove

namespace hal
{

class PIDController
{
  public:

    typedef std::function<void(double)> ControlFunc;

  public:

    PIDController(ControlFunc function) :
      setpoint(0),
      Kp(0), Ki(0), Kd(0),
      maxValue(1),
      minValue(0),
      ApplyControlUpdate(function),
      lastError(0),
      integral(0)
    {
    }

    ~PIDController()
    {
    }

    inline void Update(double feedback, double state)
    {
      const double error = setpoint - feedback;
      const double update = GetControlUpdate(error);

      std::cout << "Error:  " << error << std::endl;
      std::cout << "Update: " << update << std::endl;

      double newValue = state + update;
      newValue = std::min(maxValue, std::max(minValue, newValue));
      ApplyControlUpdate(newValue);
    }

    inline void Reset()
    {
      lastError = 0;
      integral = 0;
    }

  protected:

    inline double GetControlUpdate(double error)
    {
      double update = 0;
      update += GetProportional(error);
      update += GetIntegral(error);
      update += GetDerivative(error);
      return update;
    }

    inline double GetProportional(double error)
    {
      return Kp * error;
    }

    inline double GetIntegral(double error)
    {
      integral += error;
      return Ki * integral;
    }

    inline double GetDerivative(double error)
    {
      const double result = Kd * (error - lastError);
      lastError = error;
      return result;
    }

  public:

    double setpoint;

    double Kp;

    double Ki;

    double Kd;

    double maxValue;

    double minValue;

  protected:

    ControlFunc ApplyControlUpdate;

    double lastError;

    double integral;
};

class LuminanceImage
{
  protected:

    typedef std::function<double()> GetLuminanceFunc;

  public:

    LuminanceImage(const ImageMsg& image) :
      m_image(image),
      m_sampleCount(m_image.width() * m_image.height()),
      m_stepSize(1)
    {
      Initialize();
    }

    ~LuminanceImage()
    {
    }

    void SetMaxSampleCount(unsigned long count)
    {
      const unsigned long currentCount = m_image.width() * m_image.height();
      m_sampleCount = std::min(count, currentCount);
      m_stepSize = currentCount / m_sampleCount;
    }

    double GetLuminance() const
    {
      return GetLuminanceFromFormat();
    }

    template <typename T>
    double GetLuminanceFromMono() const
    {
      const T* data = reinterpret_cast<const T*>(m_image.data().c_str());
      double totalLuminance = 0;

      for (unsigned long i = 0; i < m_sampleCount; ++i)
      {
        totalLuminance += double(data[m_stepSize * i]);
      }

      return totalLuminance / m_sampleCount;
    }

    template <typename T>
    double GetLuminanceFromRGB() const
    {
      const T* data = reinterpret_cast<const T*>(m_image.data().c_str());
      double totalLuminance = 0;

      for (unsigned long i = 0; i < m_sampleCount; ++i)
      {
        const unsigned long index = m_stepSize * i;
        const T r = data[3 * index + 0];
        const T g = data[3 * index + 1];
        const T b = data[3 * index + 2];
        totalLuminance += GetLuminance(r, g, b);
      }

      return totalLuminance / m_sampleCount;
    }

    template <typename T>
    double GetLuminanceFromRGBA() const
    {
      const T* data = reinterpret_cast<const T*>(m_image.data().c_str());
      double totalLuminance = 0;

      for (unsigned long i = 0; i < m_sampleCount; ++i)
      {
        const unsigned long index = m_stepSize * i;
        const T r = data[4 * index + 0];
        const T g = data[4 * index + 1];
        const T b = data[4 * index + 2];
        totalLuminance += GetLuminance(r, g, b);
      }

      return totalLuminance / m_sampleCount;
    }

    template <typename T>
    double GetLuminanceFromBGR() const
    {
      const T* data = reinterpret_cast<const T*>(m_image.data().c_str());
      double totalLuminance = 0;

      for (unsigned long i = 0; i < m_sampleCount; ++i)
      {
        const unsigned long index = m_stepSize * i;
        const T b = data[3 * index + 0];
        const T g = data[3 * index + 1];
        const T r = data[3 * index + 2];
        totalLuminance += GetLuminance(r, g, b);
      }

      return totalLuminance / m_sampleCount;
    }

    template <typename T>
    double GetLuminanceFromBGRA() const
    {
      const T* data = reinterpret_cast<const T*>(m_image.data().c_str());
      double totalLuminance = 0;

      for (unsigned long i = 0; i < m_sampleCount; ++i)
      {
        const unsigned long index = m_stepSize * i;
        const T b = data[4 * index + 0];
        const T g = data[4 * index + 1];
        const T r = data[4 * index + 2];
        totalLuminance += GetLuminance(r, g, b);
      }

      return totalLuminance / m_sampleCount;
    }

    template <typename T>
    static double GetLuminance(T r, T g, T b)
    {
      return 0.2126 * r + 0.7152 * g + 0.0722 * b;
    }

  private:

    void Initialize()
    {
      switch (m_image.type())
      {
        case PB_BYTE:
          AssignLuminanceFunction<char>();
          break;

        case PB_UNSIGNED_BYTE:
          AssignLuminanceFunction<unsigned char>();
          break;

        case PB_SHORT:
          AssignLuminanceFunction<short>();
          break;

        case PB_UNSIGNED_SHORT:
          AssignLuminanceFunction<unsigned short>();
          break;

        case PB_INT:
          AssignLuminanceFunction<int>();
          break;

        case PB_UNSIGNED_INT:
          AssignLuminanceFunction<unsigned int>();
          break;

        case PB_FLOAT:
          AssignLuminanceFunction<float>();
          break;

        case PB_DOUBLE:
          AssignLuminanceFunction<double>();
          break;

        default:
          const std::string type = std::to_string(m_image.type());
          throw DeviceException("unexpected image type: " + type);
      }
    }

    template <typename T>
    void AssignLuminanceFunction()
    {
      switch (m_image.format())
      {
        case PB_LUMINANCE:
        case PB_RAW:
        {
          auto function = &LuminanceImage::GetLuminanceFromMono<T>;
          GetLuminanceFromFormat = std::bind(function, this);
          break;
        }

        case PB_RGB:
        {
          auto function = &LuminanceImage::GetLuminanceFromRGB<T>;
          GetLuminanceFromFormat = std::bind(function, this);
          break;
        }

        case PB_RGBA:
        {
          auto function = &LuminanceImage::GetLuminanceFromRGBA<T>;
          GetLuminanceFromFormat = std::bind(function, this);
          break;
        }

        case PB_BGR:
        {
          auto function = &LuminanceImage::GetLuminanceFromBGR<T>;
          GetLuminanceFromFormat = std::bind(function, this);
          break;
        }

        case PB_BGRA:
        {
          auto function = &LuminanceImage::GetLuminanceFromBGRA<T>;
          GetLuminanceFromFormat = std::bind(function, this);
          break;
        }

        default:
        {
          const std::string format = std::to_string(m_image.format());
          throw DeviceException("unexpected image format: " + format);
        }
      }
    }

  protected:

    const hal::ImageMsg& m_image;

    unsigned long m_sampleCount;

    unsigned long m_stepSize;

    GetLuminanceFunc GetLuminanceFromFormat;
};

class LightMeter
{
  public:

    LightMeter() :
      m_meanLuminance(0),
      m_meanInitialized(false),
      m_maxPixelCount(320 * 240),
      m_changeRate(1 - exp(-1))
    {
    }

    ~LightMeter()
    {
    }

    double Measure(const ImageMsg& image)
    {
      const double luminance = GetLuminance(image);
      (m_meanInitialized) ? Update(luminance) : InitialUpdate(luminance);
      return m_meanLuminance;
    }

    void Reset()
    {
      m_meanInitialized = false;
      m_meanLuminance = 0;
    }

  protected:

    double GetLuminance(const ImageMsg& image) const
    {
      LuminanceImage luminanceImage(image);
      luminanceImage.SetMaxSampleCount(m_maxPixelCount);
      return luminanceImage.GetLuminance();
    }

    void InitialUpdate(double luminance)
    {
      m_meanLuminance = luminance;
      m_meanInitialized = true;
    }

    void Update(double luminance)
    {
      m_meanLuminance += (luminance - m_meanLuminance) * m_changeRate;
    }

  protected:

    double m_meanLuminance;

    bool m_meanInitialized;

    unsigned long m_maxPixelCount;

    double m_changeRate;
};

class AutoExposureCamera : public CameraDriverInterface
{
  public:

    AutoExposureCamera() :
      m_exposureController(GetExposureControlFunc()),
      m_gainController(GetGainControlFunc()),
      m_autoExposureEnabled(false),
      m_desiredLuminance(75),
      m_changingExposure(true)
    {
      Initialize();
    }

    ~AutoExposureCamera()
    {
    }

    virtual bool GetAutoExposureEnabled() const
    {
      return m_autoExposureEnabled;
    }

    virtual void SetAutoExposureEnabled(bool enabled)
    {
      m_autoExposureEnabled = enabled;
      (m_autoExposureEnabled) ? DisableAutoExposure() : Reset();
    }

    virtual double GetDesiredLuminance() const
    {
      return m_desiredLuminance;
    }

    virtual void SetDesiredLuminance(double luminance)
    {
      m_desiredLuminance = luminance;
      m_exposureController.setpoint = m_desiredLuminance;
      m_gainController.setpoint = m_desiredLuminance;
    }

    virtual bool Capture(CameraMsg& images)
    {
      const bool success = CaptureImages(images);

      if (success && GetAutoExposureEnabled())
      {
        const int index = GetAutoExposureImageIndex();
        ImageMsg& image = *images.mutable_image(index);
        ProcessImage(image);
      }

      return success;
    }

    virtual void Reset()
    {
      m_exposureController.Reset();
      m_gainController.Reset();
      m_lightMeter.Reset();
    }

  protected:

    virtual void ProcessImage(const hal::ImageMsg& image)
    {
      const double luminance = m_lightMeter.Measure(image);
      m_gainController.maxValue = GetMaxGain();
      m_gainController.minValue = GetMinGain();

      (luminance < m_desiredLuminance) ?
          BrightenImage(luminance) : DarkenImage(luminance);
    }

    virtual void BrightenImage(double luminance)
    {
      if (GetExposure() < GetDesiredExposure())
      {
        m_exposureController.maxValue = GetDesiredExposure();
        m_exposureController.minValue = 0;
        ChangeExposure(luminance);
      }
      else
      {
        m_exposureController.maxValue = DBL_MAX;
        m_exposureController.minValue = GetDesiredExposure();
        IncreaseGain(luminance);
      }
    }

    virtual void DarkenImage(double luminance)
    {
      if (GetExposure() > GetDesiredExposure())
      {
        m_exposureController.maxValue = DBL_MAX;
        m_exposureController.minValue = GetDesiredExposure();
        ChangeExposure(luminance);
      }
      else
      {
        m_exposureController.maxValue = GetDesiredExposure();
        m_exposureController.minValue = 0;
        DecreaseGain(luminance);
      }
    }

    virtual void IncreaseGain(double luminance)
    {
      (GetGain() < GetMaxGain()) ?
        ChangeGain(luminance) : ChangeExposure(luminance);
    }

    virtual void DecreaseGain(double luminance)
    {
      std::cout << "Current Gain: " << GetGain() << ", Min Gain: " << GetMinGain() << std::endl;
      (GetGain() > GetMinGain()) ?
        ChangeGain(luminance) : ChangeExposure(luminance);
    }

    virtual void ChangeExposure(double luminance)
    {
      if (!m_changingExposure)
      {
        m_exposureController.Reset();
        m_changingExposure = true;
      }

      m_exposureController.Update(luminance, GetExposure());
    }

    virtual void ChangeGain(double luminance)
    {
      if (m_changingExposure)
      {
        m_gainController.Reset();
        m_changingExposure = false;
      }

      m_gainController.Update(luminance, GetGain());
    }

  private:

    void Initialize()
    {
      InitExposureController();
      InitGainController();
    }

    PIDController::ControlFunc GetExposureControlFunc()
    {
      using std::placeholders::_1;
      return std::bind(&AutoExposureCamera::SetExposure, this, _1);
    }

    PIDController::ControlFunc GetGainControlFunc()
    {
      using std::placeholders::_1;
      return std::bind(&AutoExposureCamera::SetGain, this, _1);
    }

    void InitExposureController()
    {
      m_exposureController.setpoint = m_desiredLuminance;
      m_exposureController.Kp = 0.8;
      m_exposureController.Ki = 0.0;
      m_exposureController.Kd = 0.0;
    }

    void InitGainController()
    {
      m_gainController.setpoint = m_desiredLuminance;
      m_gainController.Kp = 0.8;
      m_gainController.Ki = 0.0;
      m_gainController.Kd = 0.0;
    }

    virtual bool CaptureImages(hal::CameraMsg& images) = 0;

    virtual int GetAutoExposureImageIndex() = 0;

    virtual void DisableAutoExposure() = 0;

    virtual double GetDesiredExposure() = 0;

    virtual double GetExposure() = 0;

    virtual void SetExposure(double exposure) = 0;

    virtual double GetGain() = 0;

    virtual void SetGain(double gain) = 0;

    virtual double GetMaxGain() = 0;

    virtual double GetMinGain() = 0;

  protected:

    PIDController m_exposureController;

    PIDController m_gainController;

    LightMeter m_lightMeter;

  private:

    bool m_autoExposureEnabled;

    double m_desiredLuminance;

    bool m_changingExposure;
};

} // namespace hal