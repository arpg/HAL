#pragma once

#include <HAL/Camera/CameraDriverInterface.h>

#include <flycapture/FlyCapture2.h>

namespace hal {


class FlycapDriver : public CameraDriverInterface
{
    public:
        FlycapDriver();
        ~FlycapDriver();

        bool Capture( pb::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

        std::string GetDeviceProperty(const std::string& sProperty);

        size_t NumChannels() const;
        size_t Width( size_t /*idx*/ = 0 ) const;
        size_t Height( size_t /*idx*/ = 0 ) const;

    private:
        void _CheckError( FlyCapture2::Error error );

private:
        FlyCapture2::Camera      m_Cam1;
        FlyCapture2::Camera      m_Cam2;
        unsigned int             m_nImgWidth;
        unsigned int             m_nImgHeight;

};

} /* namespace */
