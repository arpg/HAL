#include "UndistortDriver.h"

#include <PbMsgs/Image.h>

namespace hal
{

UndistortDriver::UndistortDriver(
    std::shared_ptr<CameraDriverInterface> input,
    const calibu::CameraRig& rig
    )
  : m_Input(input)
{
  const size_t num_cams = rig.cameras.size();

  m_vLuts.resize(num_cams);

  for(size_t ii=0; ii< num_cams; ++ii) {
    const calibu::CameraModel& cmod = rig.cameras[ii].camera;

    // Allocate memory for LUTs.
    m_vLuts[ii] = calibu::LookupTable(cmod.Width(), cmod.Height());

    // Setup new camera model
    // For now, assume no change in scale so return same params with
    // no distortion.
    calibu::CameraModelT<calibu::Pinhole> new_cam(cmod.Width(), cmod.Height());
    new_cam.Params() << cmod.K()(0,0), cmod.K()(1,1), cmod.K()(0,2), cmod.K()(1,2);
    m_CamModel.push_back(new_cam);

    calibu::CreateLookupTable(rig.cameras[ii].camera, new_cam.Kinv(), m_vLuts[ii]);
  }
}

bool UndistortDriver::Capture( pb::CameraMsg& vImages )
{
  m_InMsg.Clear();
  const bool success = m_Input->Capture( m_InMsg );

  // Sanity check.
  if (static_cast<size_t>(m_InMsg.image_size()) > m_CamModel.size()) {
    fprintf(stderr, "HAL: Error! Expecting %d images but captured %d\n",
             static_cast<int>(m_CamModel.size()), m_InMsg.image_size());
    return false;
  }

  if(success) {
    for (int ii = 0; ii < m_InMsg.image_size(); ++ii) {
      pb::Image inimg = pb::Image(m_InMsg.image(ii));
      pb::ImageMsg* pimg = vImages.add_image();
      pimg->set_width(inimg.Width());
      pimg->set_height(inimg.Height());
      pimg->set_type( (pb::Type)inimg.Type());
      pimg->set_format( (pb::Format)inimg.Format());
      pimg->mutable_data()->resize(inimg.Width()*inimg.Height());

      pb::Image img = pb::Image(*pimg);
      calibu::Rectify(
            m_vLuts[ii], inimg.data(),
            reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
            img.Width(), img.Height());
    }
  }

  return success;
}

std::string UndistortDriver::GetDeviceProperty(const std::string& sProperty)
{
  return m_Input->GetDeviceProperty(sProperty);
}

size_t UndistortDriver::NumChannels() const
{
  return m_CamModel.size();
}

size_t UndistortDriver::Width( size_t idx ) const
{
  if(idx < m_CamModel.size()) {
    return m_CamModel[idx].Width();
  }
  return m_CamModel[0].Width();
}

size_t UndistortDriver::Height( size_t idx ) const
{
  if(idx < m_CamModel.size()) {
    return m_CamModel[idx].Height();
  }
  return m_CamModel[0].Height();
}

}
