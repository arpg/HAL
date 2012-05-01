#ifndef _DVI2PCI_H_
#define _DVI2PCI_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

#include "SDK/includes/v2u_lib.h"
#include "SDK/includes/frmgrab.h"

#include <queue>


struct DviImageBufferStruct
{
    unsigned char * m_pImageBuffer;
    unsigned char * m_pControlBuffer;
};

class Dvi2PciDriver : public CameraDriver
{
    public:
        Dvi2PciDriver();
        virtual ~Dvi2PciDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        FrmGrabber* OpenGrabber(const char* sn, const char* addr);
        static void CaptureFuncHandler(Dvi2PciDriver *pUserData);
        int DetectVideoMode(FrmGrabber* fg, V2U_VideoMode* vm, V2U_BOOL details);
        void DumpVgaModeFlags(V2U_UINT32 flags, V2U_UINT32 mask);
        bool CaptureFunc();
        bool Init();
    private:
        unsigned int                    m_nNumImages;
        unsigned int                    m_nImageWidth;
        unsigned int					m_nImageHeight;
        unsigned int m_nBufferCount;
        double                    m_dFps;

       
        FrmGrabber* m_pFrameGrabber;
        
        pthread_mutex_t m_mutex;
        
        std::queue<DviImageBufferStruct *> m_pUsedBuffers;
        std::queue<DviImageBufferStruct *> m_pFreeBuffers;
        

};

#endif
