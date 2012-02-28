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
        unsigned int                                m_nNumCams;
//        std::vector< Eigen::Matrix<double,6,1> >    m_vExtrinsics;
        unsigned int                                m_nViewportWidth;
        unsigned int                                m_nViewportHeight;
        double                                      m_dZNear;
        double                                      m_dZFar;

};

#endif
