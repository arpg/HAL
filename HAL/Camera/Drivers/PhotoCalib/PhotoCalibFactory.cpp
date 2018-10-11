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
        { "out", "8U", "Output image type: 8U, 16U, 32F" }
      };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri) override
    {
      calibu::PhotoRigd calib;
      ReadPhotoRig(uri, calib);

      std::shared_ptr<CameraDriverInterface> input = CreateInput(uri);
      const Type type = GetImageType(uri);

      return std::make_shared<PhotoCalibDriver>(input, calib, type);
    }

  protected:

    std::shared_ptr<CameraDriverInterface> CreateInput(const Uri& uri) const
    {
      return DeviceRegistry<CameraDriverInterface>::Instance().Create(uri.url);
    }

    void ReadPhotoRig(const Uri& uri, calibu::PhotoRigd& rig) const
    {
      const std::string file = uri.properties.Get<std::string>("file", "");
      calibu::PhotoRigReader reader(file);
      reader.Read(rig);
    }

    Type GetImageType(const Uri& uri) const
    {
      const std::string type = uri.properties.Get<std::string>("out", "8U");

      if (type ==  "8U") return Type::PB_UNSIGNED_BYTE;
      if (type == "16U") return Type::PB_UNSIGNED_SHORT;
      if (type == "32F") return Type::PB_FLOAT;

      std::cerr << "HAL: unsupported output image type: " << type << std::endl;
      std::cerr << "HAL: using default image type: 8U" << std::endl;
      return Type::PB_UNSIGNED_BYTE;
    }
};

static PhotoCalibFactory g_PhotoCalibFactory("photo");

} // namespace hal