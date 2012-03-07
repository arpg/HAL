#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <Eigen/Eigen>
#include "MochaException.h"
#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "Lqr.h"
 #include <iostream>
#include "Common.h"

class Controller : public boost::mutex
{
private:
    Eigen::Matrix<double,6,1> m_CarPose;
    Eigen::Matrix<double,6,1> m_LastRawPose;
    Eigen::Matrix<double,6,1> m_PoseOffset;
    bool m_bFirstPose;

    Lqr * m_pPitchController;
    Lqr * m_pRollController;
    Lqr * m_pYawController;


public:
    Controller();
    ~Controller();
    void Initialize();
    void _SetCarPose(Eigen::Matrix<double,6,1> pose);
    void Control(eLocType location);

};

#endif // CONTROLLER_H
