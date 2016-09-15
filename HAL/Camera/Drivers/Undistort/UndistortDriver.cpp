#include "UndistortDriver.h"

#include <HAL/Messages/Image.h>

namespace hal
{

UndistortDriver::UndistortDriver(std::shared_ptr<CameraDriverInterface> input,
    const std::shared_ptr<calibu::Rig<double> > rig
    )
  : m_Input(input)
{
  const size_t num_cams = rig->NumCams();

  m_vLuts.resize(num_cams);

  for(size_t ii=0; ii< num_cams; ++ii) {
    const std::shared_ptr<calibu::CameraInterface<double>> cmod = rig->cameras_[ii];

    // Allocate memory for LUTs.
    m_vLuts[ii] = calibu::LookupTable(cmod->Width(), cmod->Height());

    // Setup new camera model
    // For now, assume no change in scale so return same params with
    // no distortion.
    Eigen::Vector2i size_;
    Eigen::VectorXd params_(static_cast<int>(calibu::LinearCamera<double>::NumParams));
    size_ << cmod->Width(), cmod->Height();
    params_ << cmod->K()(0,0), cmod->K()(1,1), cmod->K()(0,2), cmod->K()(1,2);
    std::shared_ptr<calibu::CameraInterface<double>> new_cam(new calibu::LinearCamera<double>(params_, size_));
    m_CamModel.push_back(new_cam);

    calibu::CreateLookupTable(rig->cameras_[ii], new_cam->K().inverse(), m_vLuts[ii]);
  }
}

bool UndistortDriver::Capture( hal::CameraMsg& vImages )
{
  m_InMsg.Clear();
  const bool success = m_Input->Capture( m_InMsg );

  // Sanity check.
  if (static_cast<size_t>(m_InMsg.image_size()) > m_CamModel.size()) {
    fprintf(stderr, "HAL: Error! Expecting %d images but captured %d\n",
             static_cast<int>(m_CamModel.size()), m_InMsg.image_size());
    return false;
  }

  //Transfer the container's timestamps
  vImages.set_device_time(m_InMsg.device_time());
  vImages.set_system_time(m_InMsg.system_time());
  
  if(success) {
    for (int ii = 0; ii < m_InMsg.image_size(); ++ii) {
  
      hal::Image inimg = hal::Image(m_InMsg.image(ii));
      hal::ImageMsg* pimg = vImages.add_image();
      pimg->set_width(inimg.Width());
      pimg->set_height(inimg.Height());

      pimg->set_type( (hal::Type)inimg.Type());
      pimg->set_format( (hal::Format)inimg.Format());
      
      //Transfer the timestamps from the source to the destination
      pimg->set_timestamp(inimg.Timestamp());
      
      uint num_channels = 1;
      if (pimg->format() == hal::PB_LUMINANCE) {
        num_channels = 1;
      } else if (pimg->format() == hal::PB_BGRA ||
                 pimg->format() == hal::PB_RGBA) {
        num_channels = 4;
      } else {
        num_channels = 3;
      }

      hal::Image img = hal::Image(*pimg);

      if (pimg->type() == hal::PB_UNSIGNED_BYTE) {
        pimg->mutable_data()->resize(inimg.Width() * inimg.Height() *
                                     sizeof(unsigned char) * num_channels);
        calibu::Rectify<unsigned char>(
              m_vLuts[ii], inimg.data(),
              reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
              img.Width(), img.Height(), num_channels);
      } else if (pimg->type() == hal::PB_FLOAT) {
        pimg->mutable_data()->resize(inimg.Width() * inimg.Height() *
                                     sizeof(float) * num_channels);
        calibu::Rectify<float>(
              m_vLuts[ii], (float*)inimg.data(),
              reinterpret_cast<float*>(&pimg->mutable_data()->front()),
              img.Width(), img.Height(), num_channels);
      }
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
    return m_CamModel[idx]->Width();
  }
  return m_CamModel[0]->Width();
}

size_t UndistortDriver::Height( size_t idx ) const
{
  if(idx < m_CamModel.size()) {
    return m_CamModel[idx]->Height();
  }
  return m_CamModel[0]->Height();
}

}
