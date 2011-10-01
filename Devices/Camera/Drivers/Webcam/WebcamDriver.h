#ifndef _WEBCAM_H_
#define _WEBCAM_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"
#include "opencv2/highgui/highgui.hpp"

class WebcamDriver : public CameraDriver
{
    public:
        WebcamDriver();
        virtual ~WebcamDriver();
        bool Capture( std::vector<cv::Mat>& vImages );
        bool Init();
    private:
	cv::VideoCapture m_pCam;
};

#endif
