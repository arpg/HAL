#include "RectifyDriver.h"

#include <PbMsgs/Image.h>
#include <calibu/cam/StereoRectify.h>

namespace hal
{

inline float lerp(unsigned char a, unsigned char b, float t)
{
    return (float)a + t*((float)b-(float)a);
}

RectifyDriver::RectifyDriver(
        std::shared_ptr<CameraDriverInterface> input, 
        const calibu::CameraRig& rig
        )
    : m_input(input)
{
    m_vLuts.resize(rig.cameras.size());
    for(size_t i=0; i< rig.cameras.size(); ++i) {
        const calibu::CameraModel& cam = rig.cameras[i].camera;
        m_vLuts[i] = calibu::LookupTable(cam.Width(), cam.Height());
    }

    if(rig.cameras.size() == 2) {
        // Generate lookup tables for stereo rectify
        m_cam = calibu::CreateScanlineRectifiedLookupAndCameras(
                rig.cameras[1].T_wc.inverse()* rig.cameras[0].T_wc,
                rig.cameras[0].camera, rig.cameras[1].camera,
                m_T_nr_nl,
                m_vLuts[0], m_vLuts[1]
                );
    }
}

bool RectifyDriver::Capture( pb::CameraMsg& vImages )
{
    pb::CameraMsg vIn;

    const bool success = m_input->Capture( vIn );

    if(success) {
        vImages.Clear();

        pb::Image inimg[2] = { vIn.mutable_image(0), vIn.mutable_image(1)};

        for(int k=0; k < 2; ++k) {
            pb::ImageMsg* pimg = vImages.add_image();
            pimg->set_width(inimg[k].Width());
            pimg->set_height(inimg[k].Height());
            pimg->set_type( (pb::Type)inimg[k].Type());
            pimg->set_format( (pb::Format)inimg[k].Format());
            pimg->mutable_data()->resize(inimg[k].Width()*inimg[k].Height());

            pb::Image img = pb::Image(pimg);
            calibu::Rectify( m_vLuts[k], inimg[k].data(), 
                    img.data(), img.Width(), img.Height() );
        }
    }

    return success;
}

std::string RectifyDriver::GetDeviceProperty(const std::string& sProperty)
{
    return m_input->GetDeviceProperty(sProperty);
}

size_t RectifyDriver::NumChannels() const
{
    return 2;
}

size_t RectifyDriver::Width( size_t idx ) const
{
    return m_input->Width(idx);
}

size_t RectifyDriver::Height( size_t idx ) const
{
    return m_input->Height(idx);
}

}
