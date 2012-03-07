#ifndef TRAJECTORYCONTROLLER_H
#define TRAJECTORYCONTROLLER_H
#include <Eigen/Eigen>
#include "Common.h"
class TrajectoryTracker
{
private:
    double m_Kx;
    double m_Ky;
    double m_Kt;

public:
    TrajectoryTracker(double Kx, double Ky, double Kt);
    Eigen::Vector2d  UpdateControl(const Eigen::Vector5d& dCurPose, const Eigen::Vector5d& dDesiredPose);
};

#endif // TRAJECTORYCONTROLLER_H
