#ifndef SIMROBOT_H
#define SIMROBOT_H

//#include <Network/Messages.h>
#include <URDFParser/TinyXMLTool.h>
#include <BulletStructs/ModelNode.h>
//#include <Utils/ConvertName.h>


/*********************************************
  *
  * SIM ROBOT
  * SimRobot is an amalgamation of ModelNodes (Shapes, Constraints, and
  * RaycastVehicles) that make up a Robot. It has a unique name for the sum of
  * its parts, and also a 'Base' ModelNode that gives us a part from which to
  * measure a relative frame.
  * It also holds its URDF info, just in case we're through a server.
  *
  *********************************************/

class SimRobot
{
public:

  SimRobot(){
    new_parts_bit_ = 0;
    statekeeper_status_ = false;
  }

  /********************
   * SETTERS
   ********************/

  void SetName(std::string sName){
    robot_name_ = sName;
  }

  void SetBase(ModelNode* Base){
    base_part_ = Base;
  }

  void SetParts(std::vector<ModelNode*> vParts){
    if (parts_list_.size() == 0) {
      parts_list_ = vParts;
    } else {
      int size = parts_list_.size();
      SetNewPartsBit(size);
      parts_list_.insert( parts_list_.end(), vParts.begin(), vParts.end() );
    }
  }

  void SetNewPartsBit(int bit){
    new_parts_bit_ = bit;
  }

  void SetProxyName(std::string sProxyName){
    sim_name_ = sProxyName;
  }

  void SetRobotURDF(XMLDocument* pRobotURDF){
    robot_urdf_ = pRobotURDF;
  }

  /********************
   * GETTERS
   ********************/

  string GetRobotName() const {
    return robot_name_;
  }

  XMLDocument* GetRobotURDF(){
    return robot_urdf_;
  }

  std::vector<std::string> GetAllEntityNames(){
    std::vector<std::string> parts;
    for (unsigned int ii = 0; ii < parts_list_.size(); ii++) {
      parts.push_back(parts_list_.at(ii)->GetName());
    }
    return parts;
  }

  // get the pose of robot base from model node
  Eigen::Vector6d GetRobotBasePose(){
    Eigen::Vector6d pose= base_part_->GetPose();
    return pose;
  }

//  // get name of all body that belong to the robot.
  vector<string> GetAllBodyName(){
    vector<string>  vBodyName;
    vector<string>  vAllBodyNameList = GetAllEntityNames();
    for (unsigned int i = 0; i != vAllBodyNameList.size(); i++) {
      string sFullName= vAllBodyNameList[i];
      if (GetRobotNameFromFullName(sFullName) == robot_name_ ) {
        vBodyName.push_back(sFullName);
      }
    }
    return vBodyName;
  }

  std::vector<ModelNode*> GetParts(){
    return parts_list_;
  }

  int GetNewPartsBit(){
    return new_parts_bit_;
  }

  bool GetStateKeeperStatus() const {
    return statekeeper_status_;
  }

  // Get robot name (middle and last name) from full name
  string GetRobotNameFromFullName(string sFullName){
    string sRobotName;
    int Index = sFullName.find("@");
    sRobotName = sFullName.substr(Index+1, sFullName.size()-Index);
    return sRobotName;
  }

  /*****************************/

private:
  string robot_name_;
  string sim_name_;
  std::vector<ModelNode*> parts_list_;
  bool statekeeper_status_;
  ModelNode* base_part_;
  XMLDocument*  robot_urdf_;
  Eigen::Vector6d initial_pose_;
  Eigen::Vector6d urdf_pose_;
  int new_parts_bit_;
};

#endif
