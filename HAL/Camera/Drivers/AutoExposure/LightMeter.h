#pragma once

#include <functional>
#include <HAL/Messages/ImageArray.h>

namespace hal
{

class LuminanceImage
{
  protected:

    typedef std::function<double()> GetLuminanceFunc;

  public:

    LuminanceImage(const hal::ImageMsg& image);

    ~LuminanceImage();

    void SetMaxSampleCount(unsigned long count);

    double GetLuminance() const;

  protected:

    template <typename T>
    double GetLuminanceFromMono() const;

    template <typename T>
    double GetLuminanceFromRGB() const;

    template <typename T>
    double GetLuminanceFromRGBA() const;

    template <typename T>
    double GetLuminanceFromBGR() const;

    template <typename T>
    double GetLuminanceFromBGRA() const;

    template <typename T>
    static double GetLuminance(T r, T g, T b);

  private:

    void Initialize();

    template <typename T>
    void AssignLuminanceFunction();

  protected:

    const hal::ImageMsg& m_image;

    unsigned long m_sampleCount;

    unsigned long m_stepSize;

    GetLuminanceFunc GetLuminanceFromFormat;
};

class LightMeter
{
  public:

    LightMeter();

    ~LightMeter();

    double Measure(const hal::ImageMsg& image);

    void Reset();

  protected:

    double GetLuminance(const hal::ImageMsg& image) const;

    void InitialUpdate(double luminance);

    void Update(double luminance);

  protected:

    double m_meanLuminance;

    bool m_meanInitialized;

    unsigned long m_maxPixelCount;

    double m_changeRate;
};

} // namespace hal