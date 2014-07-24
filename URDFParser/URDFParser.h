// Copyright (c) bminortx

#ifndef URDFPARSER_URDFPARSER_H_
#define URDFPARSER_URDFPARSER_H_

#include <Eigen/Eigen>
#include <miniglog/logging.h>
#include <Node/Node.h>

#include <vector>
#include <map>
#include <string>

#include "URDFParser/TinyXMLTool.h"
#include "BulletStructs/Shape.h"
#include "BulletStructs/Constraint.h"
#include "BulletStructs/SimRaycastVehicle.h"
#include "SimDevices/SimDevices.h"
#include "SimRobots/SimRobot.h"
#include "SimRobots/SimWorld.h"
#include "PbMsgs/CarPlanner.pb.h"

/////////////////////////////////
/// URDF PARSER
/// Please notice that the URDF file here is not strictly the URDF format on the
/// internet. Our version is a bit different from the formal variety; We may
/// consider support for the strict URDF format in the future.
/////////////////////////////////

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
}

class URDFParser {
 public:
  explicit URDFParser(int debug_level);
  // Parses the world for the mesh and conditions.
  bool ParseWorld(const tinyxml2::XMLDocument& pDoc,
                  std::shared_ptr<SimWorld> mSimWorld);

  void GetMeshData(tinyxml2::XMLDocument& pDoc,
                   std::shared_ptr<HeightmapShape>& map_shape);

  // ParseRobot really parses each of the robot parts, and then generates a set
  // of commands that the PhysicsEngine can use to create bullet objects.
  bool ParseRobot(tinyxml2::XMLDocument& pDoc,
                  const std::string& sProxyName,
                  std::shared_ptr<SimRobot> m_SimRobot);

  void ParseShape(std::string sRobotName, tinyxml2::XMLElement *pElement);

  void ParseJoint(std::string sRobotName, tinyxml2::XMLElement *pElement);

  SimRaycastVehicle *ParseRaycastCar(std::string sRobotName,
                                     tinyxml2::XMLElement *pElement);

  void ParseSensorShape(std::string sRobotName, tinyxml2::XMLElement *pElement);

  bool ParseCommandLineForPickSensor(std::string sCommandLine);

  std::vector<std::string> GetSceneFromString(std::string sCommandLine);

  // ParseDevices uses the information given in the Robot.xml file to create the
  // sensor views that we see later in the Sim.
  bool ParseDevices(tinyxml2::XMLDocument& pDoc,
                    const std::string sProxyName,
                    std::shared_ptr<SimDevices> m_SimDevices);

  // This method is used in StateKeeper to initialize the position of every
  // object in the LocalSim.
  bool ParseWorldForInitRobotPose(const char* filename,
                                  std::vector<Eigen::Vector6d>& vRobotInitPose);

  std::vector<std::shared_ptr<ModelNode> > GetModelNodes(
      std::map<std::string, std::shared_ptr<ModelNode> > mNodes);

  ////////////////////////////////////////

  std::map<std::string, std::shared_ptr<ModelNode> > robot_models_;
  std::map<std::string, std::shared_ptr<ModelNode> > world_models_;
  node::node node_;
  int debug_level_;
};

#endif  // URDFPARSER_URDFPARSER_H_
