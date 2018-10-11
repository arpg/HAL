#pragma once

#include <HAL/Camera/AutoExposureInterface.h>
#include <HAL/Messages/ImageArray.h>
#include <calibu/cam/camera_xml.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_rig.h>
#include <calibu/cam/rectify_crtp.h>
#include <calibu/cam/stereo_rectify.h>


#include "OniSampleUtilities.h"
#include "OpenNI.h"
#include "imageintrincs.h"
#include "SE3.h"

namespace hal {

class OpenNI2Driver : public AutoExposureInterface
{
    public:
        OpenNI2Driver(unsigned int            nWidth,
                unsigned int            nHeight,
                unsigned int            nFPS,
                bool                    bCaptureRGB,
                bool                    bCaptureDepth,
                bool                    bCaptureIR,
                bool                    bAlignDepth,
                unsigned int            nExposure,
                unsigned int            nGain,
                const std::string&      sn,
                const std::string&      scmod);

        virtual ~OpenNI2Driver();

        bool Capture( hal::CameraMsg& vImages );
        std::shared_ptr<CameraDriverInterface>
          GetInputDevice() { return std::shared_ptr<CameraDriverInterface>(); }

        std::string GetDeviceProperty( const std::string& sProperty );

        size_t NumChannels() const;
        size_t Width( size_t idx = 0 ) const;
        size_t Height( size_t idx = 0 ) const;
        bool AutoExposure() const;
        unsigned int Exposure() const;
        void SetExposure(unsigned int exposure);
        unsigned int Gain() const;
        void SetGain(unsigned int gain);

        double MaxExposure(int channel) const override;

        double MinExposure(int channel) const override;

        double MaxGain(int channel) const override;

        double MinGain(int channel) const override;

        double Exposure(int channel) override;

        void SetExposure(double exposure, int channel) override;

        double Gain(int channel) override;

        void SetGain(double gain, int channel) override;

        double ProportionalGain(int channel) const override;

        double IntegralGain(int channel) const override;

        double DerivativeGain(int channel) const override;

    private:

        // NB: this is not a multi-kinect driver.  If you need multi kinects,
        // just use many of these drivers with differnt device URIs.
  std::vector<calibu::LookupTable>      m_vLuts;
        void SoftwareAlign( hal::CameraMsg& vImages );
  std::string getSerial(const std::string& Uri) const;
  uint16_t* AutoScale(const void* src, uint32_t pixelCount);
  void setHardwareRegistrationMode(bool enable);
        unsigned int                         m_height;
        unsigned int                         m_width;
        openni::VideoFrameRef                m_depthFrame;
        openni::VideoFrameRef                m_colorFrame;
        openni::VideoFrameRef                m_irFrame;
        openni::Device                       m_device;
        openni::VideoStream                  m_depthStream;
        openni::VideoStream                  m_colorStream;
        openni::VideoStream                  m_irStream;
        openni::VideoMode                    m_depthVideoMode;
        openni::VideoMode                    m_colorVideoMode;
        openni::VideoMode                    m_irVideoMode;
        std::string                          m_sURI;
        ImageIntrinsics                      m_rRGBImgIn;
        ImageIntrinsics                      m_rDepthImgIn;
        std::shared_ptr<calibu::Rig<double>> m_pRig;
        std::vector<uint64_t>                m_SerialNos;
        bool                                 m_bHardwareAlign;
        double                               m_DepthBaseline;
        double                               m_DepthFocalLength;
        std::vector<openni::VideoStream*>    m_streams;
        std::string                          m_dev_sn;
        std::string                          m_sCameraModel;
        unsigned int                         m_exposure;
        unsigned int                         m_gain;
        bool                                 m_exposureUpdated;
        int                                  m_frame;
};

}
