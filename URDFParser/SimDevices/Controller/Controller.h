// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_CONTROLLER_CONTROLLER_H_
#define URDFPARSER_SIMDEVICES_CONTROLLER_CONTROLLER_H_

#include <string>

/*********************************
  * CONTROLLER
  * This is a superclass for all controllers
  ********************************/

class Controller {
 public:
  /// SETTERS
  void SetControllerName(std::string sControllerName) {
    controller_name_ = sControllerName;
  }

  void SetRobotName(std::string sRobotName) {
    robot_name_ = sRobotName;
  }

  // Not sure what this does... keeping for now.
  void SetProxyName(std::string sProxyName) {
    sim_name_ = sProxyName;
  }

  /// GETTERS

  std::string GetControllerName() {
    return controller_name_;
  }

  std::string GetRobotName() {
    return robot_name_;
  }

  std::string GetProxyName() {
    return sim_name_;
  }

  /// MEMBER VARIABLES
  std::string controller_name_;
  std::string sim_name_;
  std::string robot_name_;
};

#endif  // URDFPARSER_SIMDEVICES_CONTROLLER_CONTROLLER_H_
