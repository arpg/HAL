#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Common.h"
#include "CarPlanner.h"

class Trajectory
{
private:
    CarPlanner m_Planner;
public:
    Trajectory();
    CarControlMatrix UpdateControl(double t, PoseMatrix pose);
    void SetTrajectory(WaypointMatrix xi, WaypointMatrix xf);
};

#endif // TRAJECTORY_H
