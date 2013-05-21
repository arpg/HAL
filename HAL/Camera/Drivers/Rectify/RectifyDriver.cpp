#include "RectifyDriver.h"

#include <PbMsgs/Image.h>
#include <calibu/cam/StereoRectify.h>

namespace hal
{

inline float lerp(unsigned char a, unsigned char b, float t)
{
    return (float)a + t*((float)b-(float)a);
}

void Remap(
        const Lut& lookup_warp,
        const pb::Image& in,
        pb::Image& out
        )
{
    for(size_t r=0; r<in.Height(); ++r) {
        for(size_t c=0; c<in.Width(); ++c) {
            const Eigen::Vector2f p = lookup_warp(r,c);
            
            const float ix = floorf(p[0]);
            const float iy = floorf(p[1]);
            const float fx = p[0] - ix;
            const float fy = p[1] - iy;
            
            const int pl = (int)ix;
            const int pr = pl + 1;
            const int pt = (int)iy;
            const int pb = pt + 1;
            
            out(r,c) = lerp(
                        lerp( in(pt,pl), in(pt,pr), fx ),
                        lerp( in(pb,pl), in(pb,pr), fx ),
                        fy
                        );
        }
    }
}

RectifyDriver::RectifyDriver(std::shared_ptr<CameraDriverInterface> input, const calibu::CameraRig& rig)
    : m_input(input)
{
    lookups.resize(rig.cameras.size());
    for(size_t i=0; i< rig.cameras.size(); ++i) {
        const calibu::CameraModel& cam = rig.cameras[i].camera;
        lookups[i] = Lut(cam.Height(), cam.Width());
    }

    if(rig.cameras.size() == 2) {
        // Generate lookup tables for stereo rectify
        m_cam = calibu::CreateScanlineRectifiedLookupAndCameras(
                rig.cameras[1].T_wc.inverse()* rig.cameras[0].T_wc,
                rig.cameras[0].camera, rig.cameras[1].camera,
                m_T_nr_nl,
                lookups[0], lookups[1]
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
            pimg->mutable_data()->reserve(inimg[k].Width()*inimg[k].Height());

            pb::Image img = pb::Image(pimg);
            Remap(lookups[k], inimg[k], img );
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
