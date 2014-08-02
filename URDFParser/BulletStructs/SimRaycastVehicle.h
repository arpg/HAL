#ifndef SIMRAYCASTVEHICLE_H
#define SIMRAYCASTVEHICLE_H

#include "Shape.h"
#include "VehicleEnums.h"
#include <iostream>
#include <list>

/*****************************************************************
 * Contents:
 * 1. CarCommand: Class to hold the steering, acceleration and
 *    reaction wheel commands that are sent to the vehicle
 * 2. SimRaycastVehicle: Class holding the parameters of the RaycastVehicle,
 *    the command list to the vehicle, and the command functors.
 *****************************************************************/

//////////////////////////////////
/// CarCommand
//////////////////////////////////
class CarCommand
{
 public:
  CarCommand():
      m_dForce(0), m_dCurvature(0), m_dT(0),
      m_dPhi(0), m_dTorque(0,0,0), m_dTime(0){

  }

  CarCommand(const double& force, const double& curvature,
             const double& dt, const double& dPhi){
    m_dForce = force;
    m_dCurvature = curvature;
    m_dTorque<<0,0,0;
    m_dT = dt;
    m_dPhi = dPhi;
  }

  CarCommand(const double& force, const double& curvature,
             const Eigen::Vector3d& torques,
             const double& dt, const double& dPhi){
    m_dForce = force;
    m_dCurvature = curvature;
    m_dTorque = torques;
    m_dT = dt;
    m_dPhi = dPhi;
  }

  double m_dForce; // this is the forward force applied to the car in newtons
  double m_dCurvature;
  double m_dT;    // this is the time period during which the command is applied
  double m_dPhi; // this is the wheel steering angle in radians
  Eigen::Vector3d m_dTorque;
  double m_dTime;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//////////////////////////////////////////////////////////////////
/// SimRaycastVehicle
//////////////////////////////////////////////////////////////////

class SimRaycastVehicle : public Shape
{
 public:

  SimRaycastVehicle(std::string sName, std::vector<double> dParameters,
                    Eigen::Vector6d dPose){
    m_dParameters = dParameters;
    model_pose_ = dPose;
    model_name_ = sName;
    m_sBodyMesh = "NONE";
    m_sWheelMesh = "NONE";
    delay_time = 0;
    // For the RC car
    servo_range = 500.0;
    total_command_time = 0;
    max_control_delay = 0.3;
    wheel_angles.push_back(0);
    wheel_angles.push_back(0);
    driving_force = 0;
  }

  ////////////
  /// SETTERS
  ////////////

  void SetWheelPose(int i, Eigen::Vector6d wheel_pose){
    if(i==0){
      m_FLWheelPose = wheel_pose;
    }
    else if(i==1){
      m_FRWheelPose = wheel_pose;
    }
    else if(i==2){
      m_BLWheelPose = wheel_pose;
    }
    else if(i==3){
      m_BRWheelPose = wheel_pose;
    }
    else{
      std::cout<<"[SimRaycastVehicle] There's nothing to do..."<<std::endl;
    }
  }

  void SetMeshes(std::string sBodyMesh, std::string sWheelMesh,
                 std::vector<double> vBodyDim, std::vector<double> vWheelDim){
    m_sBodyMesh = sBodyMesh;
    m_sWheelMesh = sWheelMesh;
    Eigen::Vector3d body_dim;
    Eigen::Vector3d wheel_dim;
    body_dim<<vBodyDim[0], vBodyDim[1], vBodyDim[2];
    wheel_dim<<vWheelDim[0], vWheelDim[1], vWheelDim[2];
    m_BodyMeshDim = body_dim;
    m_WheelMeshDim = wheel_dim;
  }

  ////////////
  /// GETTERS
  ////////////

  std::vector<double> GetParameters(){
    return m_dParameters;
  }

  Eigen::Vector6d GetWheelPose(int i){
    if(i==0){
      return m_FLWheelPose;
    }
    else if(i==1){
      return m_FRWheelPose;
    }
    else if(i==2){
      return m_BLWheelPose;
    }
    else if(i==3){
      return m_BRWheelPose;
    }
    else{
      std::cout<<"[SimRaycastVehicle] There's nothing to do..."<<std::endl;
    }
    return Eigen::Vector6d::Identity();
  }

  std::string GetBodyMesh(){
    return m_sBodyMesh;
  }

  std::string GetWheelMesh(){
    return m_sWheelMesh;
  }

  Eigen::Vector3d GetBodyMeshDim(){
    return m_BodyMeshDim;
  }

  Eigen::Vector3d GetWheelMeshDim(){
    return m_WheelMeshDim;
  }

  //////////////////////////////////
  /// FUNCTIONS
  //////////////////////////////////

  void SetAckermanSteering(double steering){
    //get the baseline of the two wheels
    double width = 2 * (m_dParameters[Width]/2-(0.3*m_dParameters[WheelWidth]));
    double length = m_dParameters[WheelBase];

    //get the inner angle
    double inner =  atan(1.0/((1.0/tan(fabs(steering))) -
                              width/(2*length) ));
    //get the outer angle
    double outer = atan(1.0/(width/length +
                             (1.0/tan(inner))));
    inner *= _sgn(steering);
    outer *= _sgn(steering);
    //now set the wheel angles depending on the steering
    if(steering >= 0 ){
      wheel_angles.at(0) = outer;
      wheel_angles.at(1) = inner;
    }else{
      wheel_angles.at(1) = outer;
      wheel_angles.at(0) = inner;
    }
    ackerman_steering = steering;
  }

  //////////////////////////////////

  void PushDelayedControl(CarCommand& delayedCommands){
    command_list.push_front(delayedCommands);
    //add to the total command time
    total_command_time += delayedCommands.m_dT;
    //if this time is over the maximum amount, pop them off at the back
    while(total_command_time > max_control_delay &&
          (total_command_time - command_list.back().m_dT)
          > max_control_delay){
      total_command_time -= command_list.back().m_dT;
      command_list.pop_back();
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  void GetDelayedControl(double timeDelay, CarCommand& delayedCommands){
    //clamp the time delay to be > 0
    timeDelay = std::max(timeDelay,0.0);
    std::list<CarCommand>& previousCommands = command_list;
    //if there is control delay, get commands from a previous time
    double currentDelay = 0;
    std::list<CarCommand>::iterator it = previousCommands.begin();
    CarCommand* pCurrentCommand = &(*it);
    int count = 0;
    if(previousCommands.size() > 1) {
      it++; //move to the first element
      for(; it != previousCommands.end() ; it++) {
        count++;
        if( currentDelay + (*it).m_dT >= timeDelay ) {
          //interpolate between the current and next commands
          double r2 = (timeDelay - currentDelay)/(*it).m_dT;
          double r1 = 1-r2;
          delayedCommands.m_dForce =
              r1*pCurrentCommand->m_dForce + r2*(*it).m_dForce;
          delayedCommands.m_dCurvature =
              r1*pCurrentCommand->m_dCurvature + r2*(*it).m_dCurvature;
          delayedCommands.m_dPhi = r1*pCurrentCommand->m_dPhi + r2*(*it).m_dPhi;
          delayedCommands.m_dTorque =
              r1*pCurrentCommand->m_dTorque + r2*(*it).m_dTorque;
          it++;
          return;
        }
        else {
          pCurrentCommand = &(*it);
          currentDelay += pCurrentCommand->m_dT;
          if(currentDelay == timeDelay) {
            delayedCommands = *pCurrentCommand;
            return;
          }
        }
      }
    }
    else if(previousCommands.size() > 0){
      delayedCommands = previousCommands.front();
    }
    else{
      delayedCommands.m_dForce = m_dParameters[AccelOffset] * servo_range;
      delayedCommands.m_dCurvature = 0;
      delayedCommands.m_dPhi = m_dParameters[SteeringOffset] * servo_range;
      delayedCommands.m_dTorque << 0,0,0;
    }

  }

  //////////////////////////////////////////////////////////////////////////////

  void CommandCar(double steering, double force, double delta_time,
                  Eigen::Vector3d lin_vel){
    CarCommand command(force, 0, delta_time, steering);
    if(delay_time == -1 && delta_time == -1 ){
      delay_time = _Tic();
      return;
    }

    //calculate the time since the last iteration
    double dT = _Toc( delay_time );
    delay_time = _Tic();

    if( delta_time != -1 ){
      dT = delta_time;
    }

    CarCommand delayedCommands = command;
    delayedCommands.m_dT = dT;
    //    if(bNoDelay == false){
    PushDelayedControl(delayedCommands);
    //get the delayed command
    GetDelayedControl(m_dParameters[ControlDelay], delayedCommands);
    //    }
    //get the delayed commands for execution and remove the offsets
    double dCorrectedForce = delayedCommands.m_dForce -
        m_dParameters[AccelOffset]*servo_range;
    double dCorrectedPhi = delayedCommands.m_dPhi -
        m_dParameters[SteeringOffset]*servo_range;

    //D.C. motor equations:
    //torque = Pwm*Ts - slope*V
    const double stallTorque = dCorrectedForce*m_dParameters[StallTorqueCoef];
    dCorrectedForce = _sgn(stallTorque) *
        std::max(0.0,fabs(stallTorque) -
                 m_dParameters[TorqueSpeedSlope] *
                 fabs(lin_vel.norm()));
    double wheelForce = dCorrectedForce/2;
    driving_force = wheelForce;

    //now apply the offset and scale values to the force and steering commands
    dCorrectedPhi = dCorrectedPhi/(m_dParameters[SteeringCoef]*servo_range);

    //clamp the steering
    dCorrectedPhi = _SoftMinimum(
        m_dParameters[MaxSteering],
        _SoftMaximum(dCorrectedPhi,-m_dParameters[MaxSteering],10),10);

    //steering needs to be flipped due to the way RayCastVehicle works
    dCorrectedPhi *= -1;

    //rate-limit the steering
    double dRate = (dCorrectedPhi - ackerman_steering)/dT;
    //clamp the rate
    dRate = _sgn(dRate) * std::min(fabs(dRate),m_dParameters[MaxSteeringRate]);
    //apply the steering
    dCorrectedPhi = ackerman_steering+dRate*dT;
    SetAckermanSteering(dCorrectedPhi);

  }

  ///////////////////
  /// MATH
  ///////////////////
  int _sgn(double n){
    return n < 0 ? -1 : n > 0;
  }

  double _SoftMaximum(double x, double y, double multiplier){
    x *= multiplier;
    y *= multiplier;
    double maximum = std::max(x, y);
    double minimum = std::min(x, y);
    return (log1p(exp(minimum-maximum)) + maximum)/multiplier;
  }

  double _SoftMinimum(double x, double y, double multiplier){
    return -_SoftMaximum(-x,-y,multiplier);
  }

  ///////////////////
  /// TIME FUNCTIONS
  ///////////////////

  double _Tic() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec  + 1e-6 * (tv.tv_usec);
  }

  double _Toc(double dTic) {
    return _Tic() - dTic;
  }

  /// MEMBER VARIABLES

  std::vector<double> m_dParameters;
  Eigen::Vector6d m_FLWheelPose;
  Eigen::Vector6d m_FRWheelPose;
  Eigen::Vector6d m_BLWheelPose;
  Eigen::Vector6d m_BRWheelPose;
  std::string m_sBodyMesh;
  std::string m_sWheelMesh;
  Eigen::Vector3d m_BodyMeshDim;
  Eigen::Vector3d m_WheelMeshDim;
  double driving_force;
  std::vector<double> wheel_angles;
  double ackerman_steering; // Theta originally passed to car.
  double delay_time;
  double servo_range;
  std::list<CarCommand> command_list;
  double total_command_time;
  double max_control_delay;


};



#endif // SIMRAYCASTVEHICLE_H
