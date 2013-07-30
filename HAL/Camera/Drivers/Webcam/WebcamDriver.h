#pragma once

#include "HAL/Camera/CameraDriverInterface.h"

#include <opencv.hpp>

namespace hal {

class WebcamDriver : public CameraDriverInterface
{
    public:
        WebcamDriver(unsigned int nCamId, bool bForceGrey);
        ~WebcamDriver();

        bool Capture( pb::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

        size_t NumChannels() const;
        size_t Width( size_t /*idx*/ = 0 ) const;
        size_t Height( size_t /*idx*/ = 0 ) const;
private:
        size_t              m_nImgHeight;
        size_t              m_nImgWidth;
        bool                m_bForceGreyscale;
        cv::VideoCapture    m_Cam;
};

}
