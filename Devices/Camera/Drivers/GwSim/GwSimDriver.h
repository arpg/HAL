/*
   \file GwSimDriver.h

 */

#ifndef _GWSIM_H_
#define _GWSIM_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"
#include <Eigen/Eigen>

class GwSimDriver : public CameraDriver
{
    public:
        GwSimDriver();
        virtual ~GwSimDriver();
        bool Capture( std::vector<cv::Mat>& vImages );
        bool Init();
    private:
        unsigned int                                m_nNumCams;
        std::vector < Eigen::Matrix<double,6,1>     m_vExtrinsics;
        unsigned int                                m_nViewportWidth;
        unsigned int                                m_nViewportHeight;
        double                                      m_dZNear;
        double                                      m_dZFar;

};

#endif
