#pragma once

#include <vector>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

#include <m3api/xiApi.h>

namespace hal {


class XimeaDriver : public CameraDriverInterface
{
public:
  XimeaDriver(std::vector<unsigned int>& vID,
               ImageRoi ROI);
  ~XimeaDriver();

  bool Capture( pb::CameraMsg& vImages );
  std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

  std::string GetDeviceProperty(const std::string& sProperty);

  size_t NumChannels() const;
  size_t Width( size_t /*idx*/ = 0 ) const;
  size_t Height( size_t /*idx*/ = 0 ) const;

private:
  void _CheckError( XI_RETURN err, std::string place);

private:

  DWORD                               dwNumberOfDevices = 0;
  HANDLE                              xiH = NULL;
  unsigned int                        m_nImgWidth;
  unsigned int                        m_nImgHeight;

};

} /* namespace */
