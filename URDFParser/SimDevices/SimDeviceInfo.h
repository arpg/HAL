// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_SIMDEVICEINFO_H_
#define URDFPARSER_SIMDEVICES_SIMDEVICEINFO_H_

#include <URDFParser/EigenHelpers.h>

#include <vector>
#include <string>

// SimDeviceInfo is a superclass that holds basic info common
// to every SimDevice.

class SimDeviceInfo {
 public:
  //  CONSTRUCTOR
  SimDeviceInfo() {
    m_bDeviceOn = false;
    m_bHasPublished = false;
  }

  // SETTERS
  void SetDeviceName(std::string sDeviceName) {
    m_sDeviceName = sDeviceName;
  }

  void SetRobotName(std::string sRobotName) {
    m_sRobotName = sRobotName;
  }

  void SetBodyName(std::string sBodyName) {
    m_sBodyName = sBodyName;
  }

  // GETTERS
  std::string GetDeviceName() {
    return m_sDeviceName;
  }

  std::string GetRobotName() {
    return m_sRobotName;
  }

  std::string GetBodyName() {
    return m_sBodyName;
  }

  //  MEMBER VARIABLES
  std::string           m_sDeviceName;    //  Device name (i.e. LCam@RaycastVehicle)
  std::string           m_sDeviceType;    //  Type of device ('Camera')
  std::string           m_sDeviceMode;    //  Mode for device ('RGB')
  //  Physics body for the device; note that the Physics body and the device
  //  usually have the same name. A notable exception is the RGBD camera, which
  //  is composed of two cameras, each with "RGB_" and "Depth_" before their
  //  device names, respectively.
  std::string           m_sBodyName;
  std::string           m_sRobotName;     //  Name of the Robot system
  bool             m_bDeviceOn;      //  Mark if device is on or not.
  //  Used in NetworkManager. True if it published already in a given cycle.
  bool             m_bHasPublished;
  bool             m_bHasAdvertised;
  Eigen::Vector6d  m_vPose;
};

#endif  // URDFPARSER_SIMDEVICES_SIMDEVICEINFO_H_
