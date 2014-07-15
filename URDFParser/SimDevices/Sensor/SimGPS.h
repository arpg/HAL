// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_SENSOR_SIMGPS_H_
#define URDFPARSER_SIMDEVICES_SENSOR_SIMGPS_H_

#include <string>

#include "SimDevices/SimDeviceInfo.h"


// return x, y, z of a GPS sensor.
class SimGPS : public SimDeviceInfo {
 public:
  SimGPS(std::string DeviceName, std::string BodyName, std::string RobotName) {
    SetDeviceName(DeviceName);
    SetBodyName(BodyName);
    SetRobotName(RobotName);
    m_sDeviceType = "GPS";
  }

  void Update(Eigen::Vector3d pose) {
    m_CurPose = pose;
  }

  void GetPose(Eigen::Vector3d pose) {
    pose << m_CurPose[0], m_CurPose[1], m_CurPose[2];
  }

  void PrintPose() {
  }

 private:
  std::string m_sDeviceName;
  Eigen::Vector3d m_CurPose;
};

#endif  // URDFPARSER_SIMDEVICES_SENSOR_SIMGPS_H_
