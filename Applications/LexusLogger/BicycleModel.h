#pragma once
#include <math.h>
#include <HAL/Utils/TicToc.h>
#include <cstdio>
#include <iostream>

#define Steering_Coeff    1
#define Steering_offset   0
#define L_Wheel2wheel     2.5  // in meters

struct Poses{
  double x;
  double y;
  double th;
};

class Bicycle
{
private:
  double prev_time;

public:
  Bicycle(){
    prev_time = hal::Tic();;
  }
  Poses PoseReset(double curr_time)
  {
    prev_time = curr_time;
    Poses currpose;
    currpose.th = 0;
    currpose.x = 0;
    currpose.y = 0;
    return currpose;
  }

  Poses PoseUpdate(Poses prev_pose,double encoder_RR,double encoder_RL,double steering_angle,double curr_time)
  {
    steering_angle = (steering_angle-Steering_offset)*Steering_Coeff;
    // Forward velocity in m/s
    double forward_velocity = ((encoder_RL+encoder_RR)*0.5)/3.6;
    double tet_dot = forward_velocity/L_Wheel2wheel;
    if(curr_time <= prev_time){
      std::cout << "Bicycle Warning : problem with time step" << std::endl;
      return prev_pose;
    }
    double tet_sig = tet_dot/(curr_time-prev_time);
    double disp_sig = forward_velocity/(curr_time-prev_time);
    Poses new_pose;
    new_pose.x = prev_pose.x+disp_sig*cos(prev_pose.th+tet_sig);
    new_pose.y = prev_pose.y+disp_sig*sin(prev_pose.th+tet_sig);
    new_pose.th = prev_pose.th+tet_sig;
    return new_pose;
  }
};
