#ifndef _WEBCAM_H_
#define _WEBCAM_H_

#include "HAL/Camera/CameraDriverInterface.h"
#include "opencv2/highgui/highgui.hpp"

class WebcamDriver : public CameraDriver
{
    public:
        WebcamDriver();
        virtual ~WebcamDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        void PrintInfo();
        bool Init();
    private:
        bool             m_bForceGreyscale;
        cv::VideoCapture m_pCam;
};

#endif
