#pragma once

#include <HAL/Camera/CameraDriverInterface.h>

#include "SDK/includes/v2u_lib.h"
#include "SDK/includes/frmgrab.h"

namespace hal {

class EpiphanDriver : public CameraDriverInterface
{
    public:
        EpiphanDriver();
        ~EpiphanDriver();

        bool Capture( hal::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

        std::string GetDeviceProperty(const std::string& sProperty);

        size_t NumChannels() const;
        size_t Width( size_t /*idx*/ = 0 ) const;
        size_t Height( size_t /*idx*/ = 0 ) const;

    private:
        FrmGrabber* OpenGrabber(const char* sn, const char* addr);
        int DetectVideoMode(FrmGrabber* fg, V2U_VideoMode* vm, V2U_BOOL details);
        void DumpVgaModeFlags(V2U_UINT32 flags, V2U_UINT32 mask);

    private:
        unsigned int                    m_nImageWidth;
        unsigned int                    m_nImageHeight;
        double                          m_dFps;

        FrmGrabber*                     m_pFrameGrabber;
};

} /* namespace */
