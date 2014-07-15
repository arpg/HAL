// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_CONTROLLER_SIMPLECONTROLLER_H_
#define URDFPARSER_SIMDEVICES_CONTROLLER_SIMPLECONTROLLER_H_

#include <PbMsgs/Pose.pb.h>
#include <SimDevices/SimDeviceInfo.h>
#include <string>
#include "PbMsgs/SimMessages.pb.h"

class SimpleController: public SimDeviceInfo {
 public:
  SimpleController(std::string sDeviceName,
                   std::string sBodyName, std::string sRobotName) {
    SetDeviceName(sDeviceName);
    SetBodyName(sBodyName);
    SetRobotName(sRobotName);
    m_sDeviceType = "SimpleController";
  }

  ///////////////////////////////////////////////////
  void UpdateCommand(const pb::PoseMsg& Command) {
  }
};

#endif  // URDFPARSER_SIMDEVICES_CONTROLLER_SIMPLECONTROLLER_H_
