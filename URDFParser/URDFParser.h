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
// Common geometric shapes available in Bullet
#include "BulletStructs/Shape.h"
// Common joints and hinges in Bullet
#include "BulletStructs/Constraint.h"
// A four-wheeled vehicle with raycasting to calculate contact points
#include "BulletStructs/SimRaycastVehicle.h"
// Common devices and sensors found on a robot
#include "SimDevices/SimDevices.h"
// Class holding a robot's structure and functions
#include "SimRobots/SimRobot.h"
// Struct holding the conditions of the world
#include "SimRobots/SimWorld.h"
// Protobufs holding information crucial to controlling
// a robot and constructing a navigable world
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

  void GetSceneFromString(std::string sCommandLine,
                          std::vector<std::string>* scene);

  // ParseDevices uses the information given in the Robot.xml file to create the
  // sensor views that we see later in the Sim.
  bool ParseDevices(tinyxml2::XMLDocument& pDoc,
                    const std::string sProxyName,
                    std::shared_ptr<SimDevices> m_SimDevices);

  std::vector<std::shared_ptr<ModelNode> > GetModelNodes(
      std::map<std::string, std::shared_ptr<ModelNode> > mNodes);

  ////////////////////////////////////////

  std::map<std::string, std::shared_ptr<ModelNode> > robot_models_;
  std::map<std::string, std::shared_ptr<ModelNode> > world_models_;
  node::node node_;
  int debug_level_;
};

#endif  // URDFPARSER_URDFPARSER_H_
