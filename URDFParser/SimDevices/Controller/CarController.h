#ifndef CARCONTROLLER_H
#define CARCONTROLLER_H

#include "SimDevices/SimDeviceInfo.h"
#include "PbMsgs/SimMessages.pb.h"
#include "PbMsgs/NodeCar.pb.h"

class CarController: public SimDeviceInfo
{
public:

  CarController(string sDeviceName, string sBodyName, string sRobotName){
    SetDeviceName(sDeviceName);
    SetBodyName(sBodyName);
    SetRobotName(sRobotName);
    m_sDeviceType = "CarController";
  }

  void UpdateCommand(pb::VehicleMsg& Command){
    m_dSteering = Command.steering_angle();
    m_dTorque = Command.desired_force();
    delta_time = Command.command_time();
  }

  double m_dSteering;
  double m_dTorque;
  double delta_time;
};

#endif // CARCONTROLLER_H
