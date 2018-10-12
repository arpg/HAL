#include "PhotoCalibDriver.h"
#include <calibu/pcalib/response_linear.h>
#include <calibu/pcalib/vignetting_uniform.h>

namespace hal
{

////////////////////////////////////////////////////////////////////////////////

class PhotoCorrection
{
  public:

    virtual ~PhotoCorrection() { }

    virtual void Correct(ImageMsg& image) = 0;

  protected:

    template <typename T>
    static Type GetType();

    template <typename T>
    static T MaxValue()
    {
      return std::numeric_limits<T>::max();
    }
};

template <>
Type PhotoCorrection::GetType<uint8_t>()
{
  return Type::PB_UNSIGNED_BYTE;
}

template <>
Type PhotoCorrection::GetType<uint16_t>()
{
  return Type::PB_UNSIGNED_SHORT;
}

template <>
Type PhotoCorrection::GetType<float>()
{
  return Type::PB_FLOAT;
}

template <>
Type PhotoCorrection::GetType<double>()
{
  return Type::PB_DOUBLE;
}

template <>
float PhotoCorrection::MaxValue<float>()
{
  return float(1);
}

template <>
double PhotoCorrection::MaxValue<double>()
{
  return double(1);
}

////////////////////////////////////////////////////////////////////////////////

template <int channels, typename in_type, typename out_type>
class PhotoCorrectionImpl : public PhotoCorrection
{
  protected:

    typedef Eigen::Array<in_type, channels, 1> InPixel;

    typedef Eigen::Array<double, channels, 1> TempPixel;

    typedef Eigen::Array<out_type, channels, 1> OutPixel;

  public:

    PhotoCorrectionImpl(int width, int height,
        const calibu::PhotoCamerad& camera) :
      width_(width),
      height_(height)
    {
      Initialize(camera);
    }

    virtual ~PhotoCorrectionImpl()
    {
    }

    void Correct(ImageMsg& image) override
    {
      const Image src(image);

      // correct each pixel in image
      for (int y = 0; y < height_; ++y)
      {
        for (int x = 0; x < width_; ++x)
        {
          buffer_.at<OutPixel>(y, x) = GetCorrection(src, y, x);
        }
      }

      UpdateImage(image);
    }

  protected:

    inline OutPixel GetCorrection(const Image& image, int y, int x)
    {
      const InPixel value = image.Mat().at<InPixel>(y, x);
      const TempPixel response = GetResponse(value);
      const TempPixel attenuation = GetAttenuation(y, x);
      return ConvertToOutput(attenuation * response);
    }

    inline TempPixel GetResponse(const InPixel& value) const
    {
      return (std::numeric_limits<in_type>::is_integer) ?
          IndexResponse(value) : InterpolateResponse(value);
    }

    inline TempPixel IndexResponse(const InPixel& value) const
    {
      TempPixel result;

      // process each channel
      for (int i = 0; i < channels; ++i)
      {
        result[i] = responses_[value[i]][i];
      }

      return result;
    }

    inline TempPixel InterpolateResponse(const InPixel& value) const
    {
      TempPixel result;

      // process each channel
      for (int i = 0; i < channels; ++i)
      {
        result[i] = InterpolateResponse(i, value[i]);
      }

      return result;
    }

    inline double InterpolateResponse(int channel, double value) const
    {
      // rescale value
      const double x = (responses_.size() - 1) * value + 0.5;

      // compute indices
      const double i0 = int(x - 0.5);
      const double i1 = i0 + 1;

      // compute weights
      const double w0 = i1 - x;
      const double w1 = 1.0 - w0;

      // compute values
      const double v0 = responses_[i0][channel];
      const double v1 = responses_[i1][channel];

      // interpolate values
      return (w0 * v0) + (w1 * v1);
    }

    inline TempPixel GetAttenuation(int y, int x) const
    {
      const int index = y * width_ + x;
      return attenuations_[index];
    }

    inline OutPixel ConvertToOutput(const TempPixel& value) const
    {
      OutPixel result;

      // process each color channel
      for (int i = 0; i < channels; ++i)
      {
        // clamp intensity values as per type
        const double max = PhotoCorrection::MaxValue<out_type>();
        result[i] = std::max(0.0, std::min(max, value[i]));
      }

      return result;
    }

    inline void UpdateImage(ImageMsg& input)
    {
      input.set_type(GetOutputType());
      input.set_data(buffer_.data, GetOutputMemorySize());
    }

    inline size_t GetOutputMemorySize() const
    {
      return sizeof(out_type) * channels * width_ * height_;
    }

    inline Type GetOutputType() const
    {
      return PhotoCorrection::GetType<out_type>();
    }

  private:

    inline void Initialize(const calibu::PhotoCamerad& camera)
    {
      CreateOutputBuffer();
      CreateResponses(camera);
      CreateAttenuations(camera);
    }

    inline void CreateOutputBuffer()
    {
      buffer_ = cv::Mat(height_, width_, GetOpenCVOutputType());
    }

    inline int GetOpenCVOutputType() const
    {
      if (std::is_same<out_type, uint8_t>::value)
          return CV_MAKETYPE(CV_8U, channels);

      if (std::is_same<out_type, uint16_t>::value)
          return CV_MAKETYPE(CV_16U, channels);

      if (std::is_same<out_type, float>::value)
          return CV_MAKETYPE(CV_32F, channels);

      if (std::is_same<out_type, double>::value)
          return CV_MAKETYPE(CV_64F, channels);

      return CV_8UC1;
    }

    inline void CreateResponses(const calibu::PhotoCamerad& camera)
    {
      // allocate response lookup table
      const double max = PhotoCorrection::MaxValue<out_type>();
      const size_t count = GetResponseCount();
      responses_.resize(count);

      // process each channel
      for (int channel = 0; channel < channels; ++channel)
      {
        const calibu::Response<double>& response = GetResponse(channel, camera);

        // process each response value
        for (size_t i = 0; i < count; ++i)
        {
          // store response in lookup table
          const double value = double(i) / (count - 1);
          responses_[i][channel] = max * response(value);
        }
      }
    }

    inline const calibu::Response<double>& GetResponse(int channel,
        const calibu::PhotoCamerad& camera) const
    {
      // check if shared response model
      return (camera.responses.size() > 1) ?
          *camera.responses[channel] : *camera.responses.front();
    }

    inline size_t GetResponseCount()
    {
      // check if indexed or interpolated lookup table
      return (std::numeric_limits<in_type>::is_integer) ?
          size_t(PhotoCorrection::MaxValue<in_type>()) + 1 : 100;
    }

    inline void CreateAttenuations(const calibu::PhotoCamerad& camera)
    {
      // allocate attenuation lookup table
      const size_t count = width_ * height_;
      attenuations_.resize(count);

      // process each channel
      for (int channel = 0; channel < channels; ++channel)
      {
        const calibu::Vignetting<double>& vignetting =
            GetVignetting(channel, camera);

        // process each pixel in vignetting
        for (int y = 0; y < height_; ++y)
        {
          const double v = vignetting.Height() * double(y + 0.5) / (height_ - 1);

          for (int x = 0; x < width_; ++x)
          {
            // store attenuation in lookup table
            const int index = y * width_ + x;
            const double u = vignetting.Width() * double(x + 0.5) / (width_ - 1);
            const double attenuation = vignetting(u, v);

            // TODO: check attenuation value

            attenuations_[index][channel] = 1 / attenuation;
          }
        }
      }
    }

    inline const calibu::Vignetting<double>& GetVignetting(int channel,
        const calibu::PhotoCamerad& camera) const
    {
      // check if shared vignetting model
      return (camera.vignettings.size() > 1) ?
          *camera.vignettings[channel] : *camera.vignettings.front();
    }

  protected:

    int width_;

    int height_;

    cv::Mat buffer_;

    std::vector<TempPixel> responses_;

    std::vector<TempPixel> attenuations_;
};

////////////////////////////////////////////////////////////////////////////////

PhotoCalibDriver::PhotoCalibDriver(std::shared_ptr<CameraDriverInterface> input,
    calibu::PhotoRigd& rig, Type type) :
  input_(input),
  rig_(rig),
  out_type_(type)
{
  Initialize();
}

bool PhotoCalibDriver::Capture(CameraMsg& images)
{
  const bool success = input_->Capture(images);
  if (success) Correct(images);
  return success;
}

std::shared_ptr<CameraDriverInterface> PhotoCalibDriver::GetInputDevice()
{
  return input_;
}

size_t PhotoCalibDriver::NumChannels() const
{
  return input_->NumChannels();
}

size_t PhotoCalibDriver::Width(size_t index) const
{
  return input_->Width(index);
}

size_t PhotoCalibDriver::Height(size_t index) const
{
  return input_->Height(index);
}

void PhotoCalibDriver::Correct(CameraMsg& images)
{
  // process each image
  for (int i = 0; i < images.image_size(); ++i)
  {
    // check if correction needed
    if (ShouldCorrect(i))
    {
      // apply correction
      ImageMsg& image = *images.mutable_image(i);
      corrections_[i]->Correct(image);
    }
  }
}

bool PhotoCalibDriver::ShouldCorrect(int index) const
{
  return corrections_[index] != nullptr;
}

void PhotoCalibDriver::Initialize()
{
  CaptureFormats();
  ValidateResponse();
  ValidateVignetting();
  CreateCorrections();
}

void PhotoCalibDriver::CaptureFormats()
{
  // capture first frame
  CameraMsg images;
  input_->Capture(images);

  // allocate type-format arrays
  in_types_.resize(images.image_size());
  in_formats_.resize(images.image_size());

  // record each type-format
  for (int i = 0; i < images.image_size(); ++i)
  {
    const ImageMsg& image = images.image(i);
    in_formats_[i] = image.format();
    in_types_[i] = image.type();
  }
}

void PhotoCalibDriver::ValidateResponse()
{
  // validate each camera
  for (std::shared_ptr<calibu::PhotoCamerad> camera : rig_.cameras)
  {
    // check if valid camera
    if (camera == nullptr) continue;

    // check if additional response needed
    if (camera->responses.empty() || camera->responses.size() == 2)
    {
      // add linear response
      std::shared_ptr<calibu::LinearResponse<double>> response;
      response = std::make_shared<calibu::LinearResponse<double>>();
      camera->responses.push_back(response);
    }
  }
}

void PhotoCalibDriver::ValidateVignetting()
{
  // validate each camera
  for (size_t i = 0; i < rig_.cameras.size(); ++i)
  {
    // check if valid camera
    std::shared_ptr<calibu::PhotoCamerad> camera = rig_.cameras[i];
    if (camera == nullptr) continue;

    // check if additional vignetting needed
    if (camera->vignettings.empty() || camera->vignettings.size() == 2)
    {
      // add uniform vignetting
      const int w = input_->Width(i);
      const int h = input_->Height(i);
      std::shared_ptr<calibu::UniformVignetting<double>> vignetting;
      vignetting = std::make_shared<calibu::UniformVignetting<double>>(w, h);
      camera->vignettings.push_back(vignetting);
    }
  }
}

void PhotoCalibDriver::CreateCorrections()
{
  bool correcting = false;

  // resize correction vector
  const int count = in_types_.size();
  corrections_.resize(count);

  // process each correction
  for (int i = 0; i < count; ++i)
  {
    // create correction if needed
    corrections_[i] = CorrectionNeeded(i) ? CreateCorrection(i) : nullptr;
    if (corrections_[i]) correcting = true;
  }

  if (!correcting)
  {
    std::cerr << "HAL: no photometric correction being performed" << std::endl;
  }
}

bool PhotoCalibDriver::CorrectionNeeded(int index) const
{
  // check if camera calibration at index
  if (index >= int(rig_.cameras.size()) || !rig_.cameras[index]) return false;

  // check all response models in camera
  for (auto response : rig_.cameras[index]->responses)
  {
    // check if non-trivial response model
    if (response && response->Type() != "linear") return true;
  }

  // check all vignetting models in camera
  for (auto vignetting : rig_.cameras[index]->vignettings)
  {
    // check if non-trivial vignettingg model
    if (vignetting && vignetting->Type() != "uniform") return true;
  }

  return false;
}

std::shared_ptr<PhotoCorrection> PhotoCalibDriver::CreateCorrection(
    size_t index) const
{
  switch (in_formats_[index])
  {
    case Format::PB_LUMINANCE:
      return CreateCorrection<1>(index);
    case Format::PB_RGB:
      return CreateCorrection<3>(index);
    case Format::PB_BGR:
      return CreateCorrection<3>(index);
    default:
      std::cerr << "HAL: unsupported input type" << std::endl;
      return nullptr;
  }
}

template <int channels>
std::shared_ptr<PhotoCorrection> PhotoCalibDriver::CreateCorrection(
    size_t index) const
{
  switch (in_types_[index])
  {
    case Type::PB_UNSIGNED_BYTE:
      return CreateCorrection<channels, uint8_t>(index);
    case Type::PB_UNSIGNED_SHORT:
      return CreateCorrection<channels, uint16_t>(index);
    case Type::PB_FLOAT:
      return CreateCorrection<channels, float>(index);
    default:
      std::cerr << "HAL: unsupported input type" << std::endl;
      return nullptr;
  }
}

template <int channels, typename in_type>
std::shared_ptr<PhotoCorrection> PhotoCalibDriver::CreateCorrection(
    int index) const
{
  switch (out_type_)
  {
    case Type::PB_UNSIGNED_BYTE:
      return CreateCorrection<channels, in_type, uint8_t>(index);
    case Type::PB_UNSIGNED_SHORT:
      return CreateCorrection<channels, in_type, uint16_t>(index);
    case Type::PB_FLOAT:
      return CreateCorrection<channels, in_type, float>(index);
    default:
      std::cerr << "HAL: unsupported output type" << std::endl;
      return nullptr;
  }
}

template <int channels, typename in_type, typename out_type>
std::shared_ptr<PhotoCorrection> PhotoCalibDriver::CreateCorrection(
    int index) const
{
  const int w = input_->Width(index);
  const int h = input_->Height(index);
  PhotoCorrectionImpl<channels, in_type, out_type>* pointer;
  std::shared_ptr<const calibu::PhotoCamerad> camera = rig_.cameras[index];
  pointer = new PhotoCorrectionImpl<channels, in_type, out_type>(w, h, *camera);
  return std::shared_ptr<PhotoCorrection>(pointer);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace hal