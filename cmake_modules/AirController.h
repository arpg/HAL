#ifndef AIRCONTROLLER_H
#define AIRCONTROLLER_H
#include "Lqr.h"
#include "Common.h"

class AirController
{
private:
    Lqr * m_pPitchController;
    Lqr * m_pRollController;
    Lqr * m_pYawController;

    Eigen::Vector3d m_vLastPose;
    double m_dLastTick;

public:
    AirController();
    void StartControl(const Eigen::Vector3d& vDesiredOrientation, double duration);
    void SetCurrentOrientation(const Eigen::Vector3d& vCurOrientation);
    Eigen::Vector3d GetControlInputs();

};

#endif // AIRCONTROLLER_H
