#ifndef SIMGPS_H
#define SIMGPS_H

#include "SimDevices/SimDeviceInfo.h"


// return x,y,z of a GPS sensor.
class SimGPS : public SimDeviceInfo
{
public:
    SimGPS(string DeviceName, string BodyName, string RobotName){
        SetDeviceName(DeviceName);
        SetBodyName(BodyName);
        SetRobotName(RobotName);
        m_sDeviceType = "GPS";
    }

    void Update(Eigen::Vector3d pose){
      m_CurPose = pose;
    }

    void GetPose(Eigen::Vector3d &pose){
        pose<<m_CurPose[0], m_CurPose[1],m_CurPose[2];
    }

    void PrintPose(){
        cout<<"[GPS PrintPose] current pose is x:"<<
              m_CurPose[0]<<" y:"<<m_CurPose[1]<<" z:"<<m_CurPose[2]<<endl;
    }

private:
    string                        m_sDeviceName;
    Eigen::Vector3d               m_CurPose;
};

#endif // SIMGPS_H
