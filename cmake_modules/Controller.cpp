#include "Controller.h"

Controller::Controller()
{
    m_bFirstPose = false;

    //zero all matrices
    m_CarPose = Eigen::Matrix<double,6,1>::Zero();
    m_LastRawPose = Eigen::Matrix<double,6,1>::Zero();
    m_PoseOffset = Eigen::Matrix<double,6,1>::Zero();
}

Controller::~Controller()
{

}

void Controller::Initialize()
{

}

void Controller::_SetCarPose(Eigen::Matrix<double,6,1> pose)
{
    //turn the pose into continuous angles

    //if this is the first time we're receiving pose data no need to correct angles
    if( m_bFirstPose == true )
    {
        m_LastRawPose = pose;
        m_CarPose = pose;
    }else
    {
        //for all three angles p,q and r
        for(int i = 0 ; i < 3 ; i++)
        {
            if(abs(pose[i] - m_LastRawPose[i]) > M_PI)
            {
                if(pose[i]>0)
                    m_PoseOffset[i] -= 2*M_PI;
                else
                    m_PoseOffset[i] += 2*M_PI;

            }
        }
        //move the last yaw value forward
        m_LastRawPose = pose;

        //set the actual car pose with the offsets
        m_CarPose = pose + m_PoseOffset;
    }
}

//void Controller::Control(eLocType location, const Eigen::Vector5d& dCurPose, const Eigen::Vector5d& dDesiredPose)
//{
//    _SetCarPose(dCurPose);

//    if(location == VT_AIR)
//    {

//    }
//}

