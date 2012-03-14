/*
   \file SimCamDriver.h

 */

#ifndef _SIMCAM_H_
#define _SIMCAM_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"
#include <Eigen/Eigen>
#include <vector>

class SimCamDriver : public CameraDriver
{
    public:
        SimCamDriver();
        virtual ~SimCamDriver();
        bool Capture( std::vector<cv::Mat>& vImages );
        bool Init();
    private:
        unsigned int                        m_nNumCams;
        std::vector< Eigen::Matrix3d >		m_vIntrinsics; // K matrices
        std::vector< Eigen::Matrix4d >		m_vExtrinsics;
//		CameraMan*							m_pCamMan;

};

#endif
