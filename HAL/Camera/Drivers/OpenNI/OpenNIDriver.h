#pragma once

#include "HAL/Camera/CameraDriverInterface.h"
#include "HAL/Messages/ImageArray.h"
#include "calibu/cam/camera_xml.h"
#include "imageintrincs.h"
#include "SE3.h"

#pragma GCC system_header
#include <XnCppWrapper.h>


namespace hal {

class OpenNIDriver : public CameraDriverInterface
{
    public:
        OpenNIDriver(unsigned int            nWidth,
                unsigned int            nHeight,
                unsigned int            nFPS,
                bool                    bCaptureRGB,
                bool                    bCaptureDepth,
                bool                    bCaptureIR,
                bool                    bAlignDepth,
                std::string             scmod);

        virtual ~OpenNIDriver();

        bool Capture( hal::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface> GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

        std::string GetDeviceProperty(const std::string& sProperty);

        size_t NumChannels() const;
        size_t Width( size_t /*idx*/ = 0 ) const;
        size_t Height( size_t /*idx*/ = 0 ) const;

        void SoftwareAlign(hal::CameraMsg& vImages);

    private:
        unsigned int                    m_nImgHeight;
        unsigned int                    m_nImgWidth;
        xn::Context                     m_Context;
        std::vector<double>             m_DepthBaselines;
        std::vector<double>             m_DepthFocalLengths;
        std::vector<xn::DepthGenerator> m_DepthGenerators;
        std::vector<xn::ImageGenerator> m_ImageGenerators;
        std::vector<xn::IRGenerator>    m_IRGenerators;
        std::vector<uint64_t>           m_SerialNos;
        std::shared_ptr<calibu::Rig<double>> m_pRig;
        ImageIntrinsics                 m_rRGBImgIn;
        ImageIntrinsics                 m_rDepthImgIn;
        bool                            m_bSoftwareAlign;
};

}
