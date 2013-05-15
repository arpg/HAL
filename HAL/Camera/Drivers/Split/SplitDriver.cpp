#include "SplitDriver.h"

namespace hal
{

SplitDriver::SplitDriver(std::shared_ptr<CameraDriverInterface> Input, std::vector<hal::ImageRoi>& vROIs, bool bCopy)
    : m_Input(Input), m_vROIs(vROIs), m_bCopy( bCopy )
{

}

bool SplitDriver::Capture( pb::CameraMsg& vImages )
{
    pb::CameraMsg pbIn;
    bool bRet = m_Input->Capture( pbIn );

    vImages.CopyFrom(pbIn);

    if( bRet ) {

        // do split
        for( unsigned int ii = 0; ii < m_vROIs.size(); ++ii ) {
            if( m_bCopy ) {
                // TODO implement deep copy version
            } else {

            }
//            const ImageRoi& roi = rois[ii];
//            StreamInfo stm(stmin.PixFormat(), roi.w, roi.h, stmin.Pitch(), (unsigned char*)0 + roi.y * stmin.Pitch() + roi.x);

        }

    }

    return false;
}

}
