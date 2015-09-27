#pragma once

#include <memory>
#include <dc1394/conversions.h>
#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>


namespace hal
{

  class DebayerDriver : public CameraDriverInterface
  {
    public:
      DebayerDriver( 
          std::shared_ptr<CameraDriverInterface> input_cam,
          const Uri& uri 
          );

      bool Capture( hal::CameraMsg& vImages );
      std::shared_ptr<CameraDriverInterface> GetInputDriver() { return m_Input; }

      size_t NumChannels() const;
      size_t Width( size_t /*idx*/ = 0 ) const;
      size_t Height( size_t /*idx*/ = 0 ) const;

    protected:
      std::shared_ptr<CameraDriverInterface>  m_Input;
      hal::CameraMsg                           m_Message;
      unsigned int                            m_nImgWidth;
      unsigned int                            m_nImgHeight;
      unsigned int                            m_nNumChannels;
      dc1394bayer_method_t                    m_Method;
      dc1394color_filter_t                    m_Filter;
      unsigned int                            m_nDepth;

  };

}
