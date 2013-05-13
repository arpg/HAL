#pragma once

#include "HAL/Camera/CameraDriverInterface.h"

//#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC system_header
#include <XnCppWrapper.h>
//#pragma GCC diagnostic pop


namespace hal {

class OpenNIDriver : public CameraDriverInterface
{
    public:
        OpenNIDriver(
                unsigned int            nWidth,
                unsigned int            nHeight,
                unsigned int            nFPS,
                bool                    bCaptureRGB,
                bool                    bCaptureDepth,
                bool                    bCaptureIR,
                bool                    bAlignDepth
                );
        virtual ~OpenNIDriver();
        bool Capture( pb::CameraMsg& vImages );

    private:
        unsigned int                    m_nImgHeight;
        unsigned int                    m_nImgWidth;
        xn::Context                     m_Context;
        std::vector<xn::DepthGenerator> m_DepthGenerators;
        std::vector<xn::ImageGenerator> m_ImageGenerators;
        std::vector<xn::IRGenerator>    m_IRGenerators;
};

}
