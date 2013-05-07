#ifndef _WEBCAM_H_
#define _WEBCAM_H_

#include "HAL/Camera/CameraDriverInterface.h"

namespace hal {

class WebcamDriver : public CameraDriver
{
    public:
        WebcamDriver();
        virtual ~WebcamDriver();
        bool Capture( pb::CameraMsg& vImages );
        void PrintInfo();
        bool Init();
    private:
        bool             m_bForceGreyscale;
        cv::VideoCapture m_pCam;
};

}

#endif
