#include <HAL/Devices/DeviceFactory.h>
#include <calibu/pcalib/pcalib_xml.h>
#include "PhotoCalibDriver.h"

namespace hal
{

class PhotoCalibFactory : public DeviceFactory<CameraDriverInterface>
{
  public:

    PhotoCalibFactory(const std::string& name) :
      DeviceFactory<CameraDriverInterface>(name)
    {
      Params() =
      {
        { "file", "", "Photometric calibration file" },
        { "channel", "0", "Image channel to correct" },
        { "colors", "1", "Number of expected color channels" },
        { "in", "8U", "Input image bit-depth: 8U, 16U, 32F" },
        { "out", "8U", "Output image bit-depth: 8U, 16U, 32F" }
      };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri) override
    {
      calibu::PhotoCalibd calib;
      ReadCalibration(uri, calib);
      std::shared_ptr<CameraDriverInterface> input = CreateInput(uri);
      const int channel = uri.properties.Get<int>("channel", 0);
      const int colors = uri.properties.Get<int>("colors", 1);
      const std::string in = uri.properties.Get<std::string>("in", "8U");
      const std::string out = uri.properties.Get<std::string>("out", "8U");

      return std::make_shared<PhotoCalibDriver>(
          input, calib, channel, colors, in, out);
    }

  protected:

    std::shared_ptr<CameraDriverInterface> CreateInput(const Uri& uri)
    {
      return DeviceRegistry<CameraDriverInterface>::Instance().Create(uri.url);
    }

    void ReadCalibration(const Uri& uri, calibu::PhotoCalibd& calib)
    {
      const std::string file = uri.properties.Get<std::string>("file", "");
      calibu::PhotoCalibReader reader(file);
      reader.Read(calib);
    }
};

static PhotoCalibFactory g_PhotoCalibFactory("photo");

} // namespace hal