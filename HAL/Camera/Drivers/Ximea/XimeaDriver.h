#pragma once

#include <vector>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

#include <xiApi.h>

namespace hal {


class XimeaDriver : public CameraDriverInterface
{
public:
  XimeaDriver(std::vector<unsigned int>& vector_ids,
              float fps,
              int exp,
              float gain,
              XI_IMG_FORMAT mode,
              ImageRoi roi);

  ~XimeaDriver();

  bool Capture(pb::CameraMsg& images);

  std::shared_ptr<CameraDriverInterface> GetInputDevice() {
    return std::shared_ptr<CameraDriverInterface>();
  }

  std::string GetDeviceProperty(const std::string& property);

  size_t NumChannels() const;

  size_t Width(size_t /*idx*/ = 0) const;

  size_t Height(size_t /*idx*/ = 0) const;

private:
  void _CheckError(XI_RETURN err, std::string place);

private:
  size_t                              num_channels;
  std::vector<HANDLE>                 cam_handle;
  XI_IMG                              image;
  XI_IMG_FORMAT                       image_format;
  unsigned int                        image_width;
  unsigned int                        image_height;

};

} /* namespace */
