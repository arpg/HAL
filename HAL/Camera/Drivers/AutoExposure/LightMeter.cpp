#include <HAL/Camera/Drivers/AutoExposure/LightMeter.h>
#include <HAL/Devices/DeviceException.h>

namespace hal
{

LuminanceImage::LuminanceImage(const ImageMsg& image) :
  m_image(image),
  m_sampleCount(m_image.width() * m_image.height()),
  m_stepSize(1)
{
  Initialize();
}

LuminanceImage::~LuminanceImage()
{
}

void LuminanceImage::SetMaxSampleCount(unsigned long count)
{
  const unsigned long currentCount = m_image.width() * m_image.height();
  m_sampleCount = std::min(count, currentCount);
  m_stepSize = currentCount / m_sampleCount;
}

double LuminanceImage::GetLuminance() const
{
  return GetLuminanceFromFormat();
}

template <typename T>
double LuminanceImage::GetLuminanceFromMono() const
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
double LuminanceImage::GetLuminanceFromRGB() const
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
double LuminanceImage::GetLuminanceFromRGBA() const
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
double LuminanceImage::GetLuminanceFromBGR() const
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
double LuminanceImage::GetLuminanceFromBGRA() const
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
double LuminanceImage::GetLuminance(T r, T g, T b)
{
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

void LuminanceImage::Initialize()
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
void LuminanceImage::AssignLuminanceFunction()
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

LightMeter::LightMeter() :
  m_meanLuminance(0),
  m_meanInitialized(false),
  m_maxPixelCount(320 * 240),
  m_changeRate(1 - exp(-1))
{
}

LightMeter::~LightMeter()
{
}

double LightMeter::Measure(const ImageMsg& image)
{
  const double luminance = GetLuminance(image);
  (m_meanInitialized) ? Update(luminance) : InitialUpdate(luminance);
  return m_meanLuminance;
}

void LightMeter::Reset()
{
  m_meanInitialized = false;
  m_meanLuminance = 0;
}

double LightMeter::GetLuminance(const ImageMsg& image) const
{
  LuminanceImage luminanceImage(image);
  luminanceImage.SetMaxSampleCount(m_maxPixelCount);
  return luminanceImage.GetLuminance();
}

void LightMeter::InitialUpdate(double luminance)
{
  m_meanLuminance = luminance;
  m_meanInitialized = true;
}

void LightMeter::Update(double luminance)
{
  m_meanLuminance += (luminance - m_meanLuminance) * m_changeRate;
}

} // namespace hal