#include "AirController.h"

AirController::AirController()
{
    m_dLastTick = 0;
}

void AirController::StartControl(const Eigen::Vector3d& vDesiredOrientation, double duration)
{
    m_pRollController->Stop();
    m_pPitchController->Stop();
    m_pYawController->Stop();

    //setup the roll controller
    m_pRollController->ScheduleGains(duration);
    Eigen::VectorXd setPoint(2);
    setPoint << vDesiredOrientation(0), 0;
    m_pRollController->Control(setPoint);

    //setup the pitch controller
    m_pPitchController->ScheduleGains(duration);
    setPoint << vDesiredOrientation(1), 0;
    m_pPitchController->Control(setPoint);

    //setup the yaw controller
    m_pYawController->ScheduleGains(duration);
    setPoint << vDesiredOrientation(2), 0;
    m_pYawController->Control(setPoint);

    //start the control sequence
    m_pRollController->Start();
    m_pPitchController->Start();
    m_pYawController->Start();
}

void AirController::SetCurrentOrientation(const Eigen::Vector3d& vCurOrientation)
{
    Eigen::Vector3d poseDerivative;

    //need to calculate the derivative of the angles
    if( m_dLastTick == 0)
    {
        m_dLastTick = Tic();
        m_vLastPose = vCurOrientation;

        //set the derivative to 0
        poseDerivative(0) = 0;
        poseDerivative(1) = 0;
        poseDerivative(2) = 0;
    }else
    {
        double seconds = Toc(m_dLastTick);
        m_dLastTick = Tic();

        //calcualte the derivative
        poseDerivative = (vCurOrientation - m_vLastPose)/seconds;
        m_vLastPose = vCurOrientation;
    }

    Eigen::Vector2d state;
    state << vCurOrientation(0), poseDerivative(0);
    m_pRollController->SetState(state);

    state << vCurOrientation(1), poseDerivative(1);
    m_pPitchController->SetState(state);

    state << vCurOrientation(2), poseDerivative(2);
    m_pYawController->SetState(state);
}

Eigen::Vector3d AirController::GetControlInputs()
{
    Eigen::Vector3d input;
    input(0) = m_pRollController->GetInput()(0);
    input(1) = m_pPitchController->GetInput()(0);
    input(2) = m_pYawController->GetInput()(0);
}


