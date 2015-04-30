#include "RectifyDriver.h"

#include <HAL/Messages/Image.h>

namespace hal
{

inline float lerp(unsigned char a, unsigned char b, float t)
{
  return (float)a + t*((float)b-(float)a);
}

RectifyDriver::RectifyDriver(std::shared_ptr<CameraDriverInterface> input,
    const std::shared_ptr<calibu::Rig<double> > rig
    )
  : m_input(input)
{
  // Convert rig to vision frame.
  std::shared_ptr<calibu::Rig<double>> new_rig =
      calibu::ToCoordinateConvention(rig, calibu::RdfVision);

  // Generate lookup tables for stereo rectify.
  m_vLuts.resize(new_rig->NumCams());
  for(size_t i=0; i< new_rig->NumCams(); ++i) {
    m_vLuts[i] = calibu::LookupTable(new_rig->cameras_[i]->Width(), new_rig->cameras_[i]->Height());
  }

  if(new_rig->NumCams() == 2) {
    m_rig = calibu::CreateScanlineRectifiedLookupAndCameras(
        new_rig->cameras_[1]->Pose().inverse()*new_rig->cameras_[0]->Pose(),
        new_rig->cameras_[0], new_rig->cameras_[1],
        m_T_nr_nl,
        m_vLuts[0], m_vLuts[1]
        );
  }
}

bool RectifyDriver::Capture( hal::CameraMsg& vImages )
{
  hal::CameraMsg vIn;

  const bool success = m_input->Capture( vIn );

  if(success) {
    vImages.Clear();

    hal::Image inimg[2] = { hal::Image(vIn.image(0)),
                           hal::Image(vIn.image(1)) };

    vImages.set_system_time(vIn.system_time());
    vImages.set_device_time(vIn.device_time());

    for(int k=0; k < 2; ++k) {
      uint num_channels = 1;
      if (inimg[k].Format() == hal::Format::PB_BGR ||
          inimg[k].Format() == hal::Format::PB_RGB) {
        num_channels = 3;
      } else if (inimg[k].Format() == hal::Format::PB_BGRA ||
                 inimg[k].Format() == hal::Format::PB_RGBA) {
        num_channels = 4;
      }

      hal::ImageMsg* pimg = vImages.add_image();
      pimg->set_width(inimg[k].Width());
      pimg->set_height(inimg[k].Height());
      pimg->set_timestamp(inimg[k].Timestamp());
      pimg->set_type( (hal::Type)inimg[k].Type());
      pimg->set_format( (hal::Format)inimg[k].Format());
      pimg->mutable_data()->resize(inimg[k].Width() * inimg[k].Height() *
                                   num_channels);

      hal::Image img = hal::Image(*pimg);
      calibu::Rectify(
            m_vLuts[k], inimg[k].data(),
            reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
            img.Width(), img.Height(), num_channels);
    }
  }

  return success;
}

std::string RectifyDriver::GetDeviceProperty(const std::string& sProperty)
{
  return m_input->GetDeviceProperty(sProperty);
}

size_t RectifyDriver::NumChannels() const
{
  return 2;
}

size_t RectifyDriver::Width( size_t idx ) const
{
  return m_input->Width(idx);
}

size_t RectifyDriver::Height( size_t idx ) const
{
  return m_input->Height(idx);
}

}
