// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_CONTROLLER_CARCONTROLLER_H_
#define URDFPARSER_SIMDEVICES_CONTROLLER_CARCONTROLLER_H_

#include <string>
#include "SimDevices/SimDeviceInfo.h"
#include "PbMsgs/SimMessages.pb.h"
#include "PbMsgs/NodeCar.pb.h"

class CarController: public SimDeviceInfo {
 public:
  CarController(std::string sDeviceName, std::string sBodyName,
                std::string sRobotName) {
    SetDeviceName(sDeviceName);
    SetBodyName(sBodyName);
    SetRobotName(sRobotName);
    m_sDeviceType = "CarController";
  }

  void UpdateCommand(const pb::VehicleMsg& Command) {
    m_dSteering = Command.steering_angle();
    m_dTorque = Command.desired_force();
    delta_time = Command.command_time();
  }

  double m_dSteering;
  double m_dTorque;
  double delta_time;
};

#endif  // URDFPARSER_SIMDEVICES_CONTROLLER_CARCONTROLLER_H_
