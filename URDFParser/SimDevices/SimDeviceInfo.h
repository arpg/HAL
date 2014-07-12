#ifndef SIMDEVICEINFO_H
#define SIMDEVICEINFO_H

#include <vector>
#include <string>
#include <URDFParser/EigenHelpers.h>

using namespace std;

// SimDeviceInfo is a superclass that holds basic info common
// to every SimDevice.

class SimDeviceInfo
{
public:

  // CONSTRUCTOR
  SimDeviceInfo(){
    m_bDeviceOn = false;
    m_bHasPublished = false;
  }

  /// SETTERS
  void SetDeviceName(string sDeviceName){
    m_sDeviceName = sDeviceName;
  }

  void SetRobotName(string sRobotName){
    m_sRobotName = sRobotName;
  }

  void SetBodyName(string sBodyName){
    m_sBodyName = sBodyName;
  }

  /// GETTERS
  string GetDeviceName(){
    return m_sDeviceName;
  }

  string GetRobotName(){
    return m_sRobotName;
  }

  string GetBodyName(){
    return m_sBodyName;
  }

  // MEMBER VARIABLES
  string           m_sDeviceName;    // Device name (i.e. LCam@RaycastVehicle)
  string           m_sDeviceType;    // Type of device ('Camera')
  string           m_sDeviceMode;    // Mode for device ('RGB')
  // Physics body for the device; note that the Physics body and the device
  // usually have the same name. A notable exception is the RGBD camera, which
  // is composed of two cameras, each with "RGB_" and "Depth_" before their
  // device names, respectively.
  string           m_sBodyName;
  string           m_sRobotName;     // Name of the Robot system
  bool             m_bDeviceOn;      // Mark if device is on or not.
  // Used in NetworkManager. True if it published already in a given cycle.
  bool             m_bHasPublished;
  bool             m_bHasAdvertised;
  Eigen::Vector6d  m_vPose;

};


#endif // SIMDEVICEINFO_H
