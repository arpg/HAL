// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_SENSOR_SIMVICON_H_
#define URDFPARSER_SIMDEVICES_SENSOR_SIMVICON_H_

#include <string>

#include "SimDevices/SimDeviceInfo.h"

// return x, y, z, p, q, r of body A.

class SimVicon : public SimDeviceInfo {
 public:
  // request bullet the pose of object.
  void init(std::string DeviceName, std::string BodyName) {
    m_sDeviceName  = DeviceName;
    m_sBodyName  = BodyName;
  }

  void Update() {
    // update pose
    // m_Poses = m_rPhysWrapper.GetEntity6Pose(m_sBodyName);
  }

  Eigen::Vector6d GetLatestPose() {
    return m_Poses;
  }

  void PrintCurPose() {
  }

 private:
  std::string m_sDeviceName;
  std::string m_sBodyName;
  Eigen::Vector6d m_Poses;
};


#endif  // URDFPARSER_SIMDEVICES_SENSOR_SIMVICON_H_
