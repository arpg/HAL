#include "TrajectoryTracker.h"

TrajectoryTracker::TrajectoryTracker(double Kx, double Ky, double Kt)
{
    m_Kx = Kx;
    m_Ky = Ky;
    m_Kt = Kt;
}

Eigen::Vector2d TrajectoryTracker::UpdateControl(const Eigen::Vector5d& dCurPose, const Eigen::Vector5d& dDesiredPose)
{
    Eigen::Vector2d controls;

    //get the rotation matrix for the car
    Eigen::Matrix<double,3,3> r;
    r << cos(dCurPose(2)) , sin(dCurPose(2)) , 0 , -sin(dCurPose(2)) , cos(dCurPose(2)) , 0, 0, 0, 1;

    Eigen::Vector3d poseError = dDesiredPose.topLeftCorner(3,1) - dCurPose.topLeftCorner(3,1);
    //transform the error
    poseError = r*poseError;

    double vc = dDesiredPose(3)*cos(poseError(2)) + m_Kx*poseError(0);
    double wc = dDesiredPose(4) + dDesiredPose(3)*(m_Ky*poseError(1) + m_Kt*sin(poseError(2)));

    controls << vc, wc;
    return controls;
}

