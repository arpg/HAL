#include "Trajectory.h"

Trajectory::Trajectory()
{
}

void Trajectory::SetTrajectory(double t, WaypointMatrix xi, WaypointMatrix xf)
{
    //create the cubic between these two points
    m_Planner.SetGoal(t,xi,xf);

}

CarControlMatrix Trajectory::UpdateControl(double t, PoseMatrix pose)
{
    CarControlMatrix control;
    //get the current trajectory
    CurvePoseMatrix curvePose = m_Planner.UpdateTrajectory(t,pose);
    pose - curvePose
    //control[0] =

}

