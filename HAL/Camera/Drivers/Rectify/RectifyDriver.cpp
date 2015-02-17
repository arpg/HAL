#include "RectifyDriver.h"

#include <PbMsgs/Image.h>

namespace hal
{

inline float lerp(unsigned char a, unsigned char b, float t)
{
  return (float)a + t*((float)b-(float)a);
}

RectifyDriver::RectifyDriver(
    std::shared_ptr<CameraDriverInterface> input,
    const calibu::CameraRig& rig
    )
  : m_input(input)
{
  // Convert rig to vision frame.
  calibu::CameraRig new_rig;
  new_rig = calibu::ToCoordinateConvention(rig, calibu::RdfVision);

  // Generate lookup tables for stereo rectify.
  m_vLuts.resize(new_rig.cameras.size());
  for(size_t i=0; i< new_rig.cameras.size(); ++i) {
    const calibu::CameraModel& cam = new_rig.cameras[i].camera;
    m_vLuts[i] = calibu::LookupTable(cam.Width(), cam.Height());
  }

  if(rig.cameras.size() == 2) {
    m_cam = calibu::CreateScanlineRectifiedLookupAndCameras(
        new_rig.cameras[1].T_wc.inverse()* new_rig.cameras[0].T_wc,
        new_rig.cameras[0].camera, new_rig.cameras[1].camera,
        m_T_nr_nl,
        m_vLuts[0], m_vLuts[1]
        );
  }
}

bool RectifyDriver::Capture( pb::CameraMsg& vImages )
{
  pb::CameraMsg vIn;

  const bool success = m_input->Capture( vIn );

  if(success) {
    vImages.Clear();

    pb::Image inimg[2] = { pb::Image(vIn.image(0)),
                           pb::Image(vIn.image(1)) };

    vImages.set_system_time(vIn.system_time());
    vImages.set_device_time(vIn.device_time());

    for(int k=0; k < 2; ++k) {
      pb::ImageMsg* pimg = vImages.add_image();
      pimg->set_width(inimg[k].Width());
      pimg->set_height(inimg[k].Height());
      pimg->set_timestamp(inimg[k].Timestamp());
      pimg->set_type( (pb::Type)inimg[k].Type());
      pimg->set_format( (pb::Format)inimg[k].Format());
      pimg->mutable_data()->resize(inimg[k].Width()*inimg[k].Height());

      pb::Image img = pb::Image(*pimg);
      calibu::Rectify(
            m_vLuts[k], inimg[k].data(),
            reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
            img.Width(), img.Height());
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
