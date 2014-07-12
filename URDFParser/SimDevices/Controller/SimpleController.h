#ifndef SIMPLECONTROLLER_H
#define SIMPLECONTROLLER_H

#include <SimDevices/SimDeviceInfo.h>
#include "PbMsgs/SimMessages.pb.h"
#include <PbMsgs/Pose.pb.h>

class SimpleController: public SimDeviceInfo
{
public:

  SimpleController(string sDeviceName, string sBodyName, string sRobotName){
    SetDeviceName(sDeviceName);
    SetBodyName(sBodyName);
    SetRobotName(sRobotName);

    m_sDeviceType = "SimpleController";
  }

  ///////////////////////////////////////////////////

  void UpdateCommand(pb::PoseMsg& Command){
//    Eigen::Vector6d eCommand;
//    eCommand<<Command.pose(0), Command.pose(1), Command.pose(2),
//        Command.pose(3),Command.pose(4),Command.pose(5);
//    vector<string> vBodyFullName;
//    for(int j =0; j!= Command.body_name_size();j++){
//      string sBodyFullName = Command.body_name(j);
//      vBodyFullName.push_back(sBodyFullName);
//    }
//    m_eCommand.push_back(eCommand);
//    m_vBodyFullName = vBodyFullName;
//    if(eCommand[0]!=0 || eCommand[1]!=0 || eCommand[2]!=0 || eCommand[3]!=0 ||
//       eCommand[4]!=0 || eCommand[5]!=0){
//      m_bCommandIsLatestFlag = true;
//    }
//    else{
//      m_bCommandIsLatestFlag = false;
//    }
//  }

//  // mark if command is latest. if not, we will not send this command to statekeeper.
//  bool                           m_bCommandIsLatestFlag;
//  // body that will be applied latest command. In many Cases it will be pair body.
//  vector<string>                 m_vBodyFullName;
//  std::vector< Eigen::Vector6d > m_eCommand;
  }

};

#endif // SIMROBOTCONTROLLER_H
