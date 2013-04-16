#ifndef _DVI2PCI_H_
#define _DVI2PCI_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

#include "SDK/includes/v2u_lib.h"
#include "SDK/includes/frmgrab.h"


class Dvi2PciDriver : public CameraDriver
{
    public:
        Dvi2PciDriver();
        virtual ~Dvi2PciDriver();
        bool Init();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );

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

#endif
