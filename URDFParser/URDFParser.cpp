// Copyright (c) bminortx

#include "URDFParser/URDFParser.h"

#include <map>
#include <string>
#include <vector>

URDFParser::URDFParser(int debug_level) {
  debug_level_ = debug_level;
}

////////////////////////////////////////////////////////////
/// PARSE WORLD.XML IN LocalSim
////////////////////////////////////////////////////////////
bool URDFParser::ParseWorld(const tinyxml2::XMLDocument& pDoc,
                            std::shared_ptr<SimWorld> mSimWorld) {
  const tinyxml2::XMLElement *pParent = pDoc.RootElement();
  const tinyxml2::XMLElement *pElement = pParent->FirstChildElement();

  //  read high level parent (root parent)
  while (pElement) {
    const char* sRootContent = pElement->Name();
    if (strcmp(sRootContent, "base") == 0) {
      std::string sMesh(pElement->Attribute("mesh"));
      mSimWorld->mesh_dir_ = sMesh;
      mSimWorld->pose_ =
          GenNumFromChar(pElement->Attribute("worldpose"));
      mSimWorld->normal_ =
          GenNumFromChar(pElement->Attribute("worldnormal"));
      mSimWorld->robot_pose_ =
          GenNumFromChar(pElement->Attribute("robotpose"));
      std::vector<double> vLightPose =
          GenNumFromChar(pElement->Attribute("lightpose"));
      std::shared_ptr<LightShape> pLight =
          std::make_shared<LightShape>("Light", vLightPose);
      world_models_[pLight->GetName()] = pLight;
      // ** There are several options for mesh design: **//
      //  1. Init world without mesh. Creates a flat plane.
      if (mSimWorld->mesh_dir_ == "NONE") {
        LOG(debug_level_) << "World's mesh type: "
                          << mSimWorld->mesh_dir_;
        //  We can't just use a giant box here; the RaycastVehicle won't
        //  connect, and will go straight through.
        std::shared_ptr<PlaneShape> pGround =
            std::make_shared<PlaneShape>("Ground",
                                         mSimWorld->normal_,
                                         mSimWorld->pose_);
        world_models_[pGround->GetName()] = pGround;
      } else if (mSimWorld->mesh_dir_ == "MATLAB") {
        //  2. Init world from MATLAB height data.
        LOG(debug_level_) << "World's mesh type: "
                          << mSimWorld->mesh_dir_;
        node_.init("URDF");
        while (!node_.subscribe("MATLAB/Heightmap")) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        LOG(debug_level_) << "SUCCESS: Subscribed to MATLAB/Heightmap";
        pb::PlannerHeightmapMsg params;
        while (!node_.receive("MATLAB/Heightmap", params)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        LOG(debug_level_)
            << "SUCCESS: Received heightstd::map data from MATLAB";
        int row_count = params.row_count();
        int col_count = params.col_count();
        std::vector<double> X, Y, Z;
        X.resize(row_count*col_count);
        Y.resize(row_count*col_count);
        Z.resize(row_count*col_count);
        for (int ii = 0; ii < params.x_data().size(); ii++) {
          X.at(ii) = params.x_data().Get(ii);
          Y.at(ii) = params.y_data().Get(ii);
          Z.at(ii) = params.z_data().Get(ii);
        }
        std::shared_ptr<HeightmapShape> map_shape =
            std::make_shared<HeightmapShape>("map",
                                             row_count, col_count,
                                             X, Y, Z);
        world_models_[map_shape->GetName()] = map_shape;
      } else if (mSimWorld->mesh_dir_ == "CSV") {
        //  3. Fill the .xml with all of the values for the mesh file.
        //  That way, we won't have to keep importing them from MATLAB,
        //  which really takes forever.
        LOG(debug_level_) << "World's mesh type: "
                          << mSimWorld->mesh_dir_;
        double row_count = ::atof(pElement->Attribute("row_count"));
        double col_count = ::atof(pElement->Attribute("col_count"));
        std::vector<double> x_data = GenNumFromChar(
            pElement->Attribute("x_data"));
        std::vector<double> y_data = GenNumFromChar(
            pElement->Attribute("y_data"));
        std::vector<double> z_data = GenNumFromChar(
            pElement->Attribute("z_data"));
        std::vector<double> X, Y, Z;
        X.resize(row_count*col_count);
        Y.resize(row_count*col_count);
        Z.resize(row_count*col_count);
        for (int ii = 0; ii < x_data.size(); ii++) {
          X.at(ii) = x_data.at(ii);
          Y.at(ii) = y_data.at(ii);
          Z.at(ii) = z_data.at(ii);
        }
        std::shared_ptr<HeightmapShape> map_shape =
            std::make_shared<HeightmapShape>("map",
                                             row_count, col_count,
                                             X, Y, Z);
        world_models_[map_shape->GetName()] = map_shape;
      } else {
        //  4. We actually have a mesh.
        LOG(debug_level_) << "World's mesh type: mesh";
        LOG(debug_level_) << "Mesh directory: "
                          << mSimWorld->mesh_dir_;
        std::shared_ptr<MeshShape> pMesh =
            std::make_shared<MeshShape>("map",
                                        mSimWorld->mesh_dir_,
                                        mSimWorld->pose_);
        world_models_[pMesh->GetName()] = pMesh;
      }
    }
    pElement = pElement->NextSiblingElement();
  }
  LOG(debug_level_) << "SUCCESS: World initialized";
  mSimWorld->models_ = GetModelNodes(world_models_);
  return true;
}

// This function serves as a shortcut; maybe we don't want to
// have everything, you know? Just the data.
void URDFParser::GetMeshData(tinyxml2::XMLDocument& pDoc,
                             std::shared_ptr<HeightmapShape>& map_shape) {
  tinyxml2::XMLElement *pParent = pDoc.RootElement();
  tinyxml2::XMLElement *pElement = pParent->FirstChildElement();
  //  read high level parent (root parent)
  while (pElement) {
    const char* sRootContent = pElement->Name();
    if (strcmp(sRootContent, "base") == 0) {
      std::string sMesh(pElement->Attribute("mesh"));
      if (sMesh == "CSV") {
        double row_count = ::atof(pElement->Attribute("row_count"));
        double col_count = ::atof(pElement->Attribute("col_count"));
        std::vector<double> x_data = GenNumFromChar(
            pElement->Attribute("x_data"));
        std::vector<double> y_data = GenNumFromChar(
            pElement->Attribute("y_data"));
        std::vector<double> z_data = GenNumFromChar(
            pElement->Attribute("z_data"));
        map_shape = std::make_shared<HeightmapShape>("map",
                                                     row_count, col_count,
                                                     x_data, y_data, z_data);
        return;
      }
    }
  }
}

////////////////////////////////////////////////////////////
/// PARSE ROBOT.XML FOR ROBOT PARTS AND BUILD INTO LocalSim
/// The robot name format: robotname@proxyname. e.g. robot1@proxy1.
/// All devices will live under this name.
/// The name of any robot body is: BodyName@RobotName@ProxyName
/// The name of any robot joint is: JointName@RobotName@ProxyName
////////////////////////////////////////////////////////////
bool URDFParser::ParseRobot(tinyxml2::XMLDocument& pDoc,
                            const std::string& sProxyName,
                            std::shared_ptr<SimRobot> rSimRobot) {
  LOG(debug_level_) << "Parsing robot body:";
  tinyxml2::XMLElement *pParent = pDoc.RootElement();
  std::string sRobotName(GetAttribute(pParent, "name"));
  sRobotName = sProxyName;

  rSimRobot->SetName(sRobotName);
  rSimRobot->SetRobotURDF(&pDoc);
  rSimRobot->SetProxyName(sProxyName);
  tinyxml2::XMLElement *pElement = pParent->FirstChildElement();

  //  Need to do something with robot_models_; right now, nothing happens.
  //  Which is silly.
  while (pElement) {
    const char* sRootContent = pElement->Name();

    /////////////////////////////////////////
    //  THE BODY BASE
    /////////////////////////////////////////
    if (strcmp(sRootContent, "bodybase") == 0) {
      std::string sBodyName = GetAttribute(pElement, "name");
      if (sBodyName == "RaycastVehicle") {
        //  Assign the raycast vehicle as the bodybase.
        sBodyName = sBodyName+"@"+sRobotName;

        /////////////////////////////////////////
        //  Raycast Car
        /////////////////////////////////////////
        SimRaycastVehicle* pVehicle = ParseRaycastCar(sBodyName, pElement);
        rSimRobot->SetBase(pVehicle);
        LOG(debug_level_) << "SUCCESS: built base for " << sBodyName;
      } else {
        sBodyName = sBodyName+"@"+sRobotName;
        int iMass = ::atoi(pElement->Attribute("mass"));
        std::vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
        std::vector<double> vDimension =
            GenNumFromChar(pElement->Attribute("dimension"));
        const char* sType = pElement->Attribute("type");
        if (strcmp(sType, "Box") == 0) {
          std::shared_ptr<BoxShape> pBox =
              std::make_shared<BoxShape>(sBodyName, vDimension[0],
                                         vDimension[1],
                                         vDimension[2], iMass, 1, vPose);
          rSimRobot->SetBase(pBox.get());
          LOG(debug_level_) << "Successfully built base for " << sBodyName;
          robot_models_[pBox->GetName()] = pBox;
        }
      }
    }

    /////////////////////////////////////////
    //  ALL OF OUR BODIES (SHAPES)
    /////////////////////////////////////////
    //  Build shapes connected to the body base.
    ParseShape(sRobotName, pElement);

    /////////////////////////////////////////
    //  ALL OF OUR SENSOR BODIES
    /////////////////////////////////////////
    ParseSensorShape(sRobotName, pElement);

    /////////////////////////////////////////
    //  ALL OF OUR JOINTS (CONSTRAINTS)
    /////////////////////////////////////////
    ParseJoint(sRobotName, pElement);

    //  read next parent element
    pElement = pElement->NextSiblingElement();
  }

  rSimRobot->SetParts(GetModelNodes(robot_models_));

  return true;
}


////////////////////////////////////////////////////////////
/// Read command line and try to init sensor based on command line
/// The input command line looks like:
/// Openni:[Name = "LCamera", rgb = 1, depth = 1]//
/// Openni:[Name = "RCamera", rgb = 1]//
////////////////////////////////////////////////////////////
bool URDFParser::ParseCommandLineForPickSensor(std::string sCommandLine) {
  //  get scheme:
  return true;
}

// get scene for init device
void URDFParser::GetSceneFromString(std::string sCommandLine,
                                    std::vector<std::string>* scene) {
  if (scene) {
    //  get scene by looking for "//"
    while (sCommandLine.size()!= 0) {
      std::string sScene =
          sCommandLine.substr(0, sCommandLine.find_first_of("//")+1);
      if (sScene.find("//")!= std::string::npos) {
        LOG(debug_level_) << "[GetSceneFromStd::String] Get Scene: "
                          << sScene;
        scene->push_back(sScene);
      } else {
        if (sCommandLine.size() == 0) {
          return;
        } else {
          LOG(debug_level_) << "[GetSceneFromStd::String] Fatal Error!"
                            << " Invalid command line std::string "
                            << sCommandLine;
          exit(-1);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////
/// Parse Shape (Body)
////////////////////////////////////////////////////////////

void URDFParser::ParseShape(std::string sRobotName,
                            tinyxml2::XMLElement *pElement) {
  const char* sRootContent = pElement->Name();

  if (strcmp(sRootContent, "body") == 0) {
    std::string sBodyName = GetAttribute(pElement, "name")+"@"+sRobotName;
    LOG(debug_level_) << "Building " <<  sBodyName;
    int iMass = ::atoi(pElement->Attribute("mass"));
    std::vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
    std::vector<double> vDimension =
        GenNumFromChar(pElement->Attribute("dimension"));
    const char* sType = pElement->Attribute("type");

    if (strcmp(sType, "Box") == 0) {
      std::shared_ptr<BoxShape> pBox =
          std::make_shared<BoxShape>(sBodyName, vDimension[0], vDimension[1],
                                    vDimension[2], iMass, 1, vPose);
      robot_models_[pBox->GetName()] = pBox;
    } else if (strcmp(sType, "Cylinder") == 0) {
      std::shared_ptr<CylinderShape> pCylinder =
          std::make_shared<CylinderShape>(sBodyName, vDimension[0],
                                          vDimension[1], iMass, 1,
                                          vPose);
      robot_models_[pCylinder->GetName()] = pCylinder;
    } else if (strcmp(sType, "Sphere") == 0) {
      std::shared_ptr<SphereShape> pSphere =
          std::make_shared<SphereShape>(sBodyName, vDimension[0],
                                             iMass, 1, vPose);
      robot_models_[pSphere->GetName()] = pSphere;
    } else if (strcmp(sType, "Plane") == 0) {
      std::shared_ptr<PlaneShape> pPlane =
          std::make_shared<PlaneShape>(sBodyName, vDimension, vPose);
      robot_models_[pPlane->GetName()] = pPlane;
    } else if (strcmp(sType, "Mesh") == 0) {
      std::string file_dir = pElement->Attribute("dir");
      std::shared_ptr<MeshShape> pMesh =
          std::make_shared<MeshShape>(sBodyName, file_dir, vPose);
      robot_models_[pMesh->GetName()] = pMesh;
    }
    LOG(debug_level_) << "SUCCESS: Built " << sBodyName;
  }
}


////////////////////////////////////////////////////////////
/// Parse Joint
////////////////////////////////////////////////////////////

void URDFParser::ParseJoint(std::string sRobotName,
                            tinyxml2::XMLElement *pElement) {
  const char* sRootContent = pElement->Name();
  if (strcmp(sRootContent, "joint") == 0) {
    // Attributes common to all joints
    std::string sJointName = GetAttribute(pElement, "name")+"@"+sRobotName;
    std::string sJointType(pElement->Attribute("type"));

    ///////////////
    //  Hinge
    ///////////////
    if (sJointType == "HingeJoint") {
      LOG(debug_level_) << "Building Hinge";
      std::string sParentName;
      std::string sChildName;
      std::vector<double> vPivot;
      std::vector<double> vAxis;
      double dUpperLimit = M_PI;
      double dLowerLimit = 0;
      double dSoftness = .5;
      double dBias = .5;
      double dRelaxation = .5;

      //  Construct joint based on children information.
      //  This information may include links, origin, axis.etc
      tinyxml2::XMLElement *pChild = pElement->FirstChildElement();
      while (pChild) {
        const char * sName = pChild->Name();
        //  get parent body of joint
        if (strcmp(sName, "parent") == 0) {
          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        //  get child body of joint
        if (strcmp(sName, "child") == 0) {
          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        if (strcmp(sName, "pivot") == 0) {
          vPivot = GenNumFromChar(pChild->Attribute("setting"));
        }
        if (strcmp(sName, "axis") == 0) {
          vAxis = GenNumFromChar(pChild->Attribute("setting"));
        }
        if (strcmp(sName, "upperlimit") == 0) {
          dUpperLimit = ::atof(sName);
        }
        if (strcmp(sName, "lowerlimit") == 0) {
          dLowerLimit = ::atof(sName);
        }
        if (strcmp(sName, "softness") == 0) {
          dSoftness = ::atof(sName);
        }
        if (strcmp(sName, "bias") == 0) {
          dBias = ::atof(sName);
        }
        if (strcmp(sName, "relaxation") == 0) {
          dRelaxation = ::atof(sName);
        }
        //  read next child (joint)
        pChild = pChild->NextSiblingElement();
      }
      Eigen::Vector3d pivot;
      pivot << vPivot[0], vPivot[1], vPivot[2];
      Eigen::Vector3d axis;
      axis << vAxis[0], vAxis[1], vAxis[2];
      std::shared_ptr<HingeTwoPivot> pHinge = std::make_shared<HingeTwoPivot>(
          sJointName,
          static_cast<Shape*>(robot_models_.find(sParentName)->second.get()),
          static_cast<Shape*>(robot_models_.find(sChildName)->second.get()),
          pivot, Eigen::Vector3d::Zero(),
          axis, Eigen::Vector3d::Zero());
      pHinge->SetLimits(dLowerLimit, dUpperLimit,
                        dSoftness, dBias, dRelaxation);
      robot_models_[pHinge->GetName()] = pHinge;
      LOG(debug_level_) << "SUCCESS: Built Hinge";
    } else if (sJointType == "Hinge2Joint") {
      //////////////
      //  Hinge2
      //////////////
      LOG(debug_level_) << "Building Hinge2";
      std::string sParentName;
      std::string sChildName;
      std::vector<double> vAnchor;
      std::vector<double> vAxis1;
      std::vector<double> vAxis2;
      std::vector<double> vLowerLinearLimit;
      std::vector<double> vUpperLinearLimit;
      std::vector<double> vLowerAngleLimit;
      std::vector<double> vUpperAngleLimit;

      Eigen::Vector3d Anchor;
      Eigen::Vector3d Axis1;
      Eigen::Vector3d Axis2;
      Eigen::Vector3d LowerLinearLimit;
      Eigen::Vector3d UpperLinearLimit;
      Eigen::Vector3d LowerAngleLimit;
      Eigen::Vector3d UpperAngleLimit;

      //  read detail of a joint
      tinyxml2::XMLElement *pChild = pElement->FirstChildElement();

      //  Construct joint based on children information.
      //  This information may include links, origin, axis.etc
      while (pChild) {
        const char * sName = pChild->Name();
        //  get parent link of joint
        if (strcmp(sName, "parent") == 0) {
          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        //  get child link of joint
        if (strcmp(sName, "child") == 0) {
          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        if (strcmp(sName, "anchor") == 0) {
          vAnchor = GenNumFromChar(pChild->Attribute("setting"));
          Anchor << vAnchor[0], vAnchor[1], vAnchor[2];
        }
        if (strcmp(sName, "axis") == 0) {
          vAxis1 = GenNumFromChar(pChild->Attribute("axis1"));
          vAxis2 = GenNumFromChar(pChild->Attribute("axis2"));

          Axis1 << vAxis1[0], vAxis1[1], vAxis1[2];
          Axis2 << vAxis2[0], vAxis2[1], vAxis2[2];
        }
        if (strcmp(sName, "limit") == 0) {
          vLowerLinearLimit = GenNumFromChar(pChild->Attribute("lowerlinear"));
          vUpperLinearLimit = GenNumFromChar(pChild->Attribute("upperlinear"));
          vLowerAngleLimit = GenNumFromChar(pChild->Attribute("lowerangle"));
          vUpperAngleLimit = GenNumFromChar(pChild->Attribute("upperangle"));
          LowerAngleLimit << vLowerAngleLimit[0], vLowerAngleLimit[1],
              vLowerAngleLimit[2];
          UpperAngleLimit << vUpperAngleLimit[0], vUpperAngleLimit[1],
              vUpperAngleLimit[2];
          LowerLinearLimit << vLowerLinearLimit[0], vLowerLinearLimit[1],
              vLowerLinearLimit[2];
          UpperLinearLimit << vUpperLinearLimit[0], vUpperLinearLimit[1],
              vUpperLinearLimit[2];
        }
        //  read next child (joint)
        pChild = pChild->NextSiblingElement();
      }
      std::shared_ptr<Hinge2> pHinge2 = std::make_shared<Hinge2>(
          sJointName,
          static_cast<Shape*>(robot_models_.find(sParentName)->second.get()),
          static_cast<Shape*>(robot_models_.find(sChildName)->second.get()),
          Anchor, Axis1, Axis2);
      pHinge2->SetLimits(1, 1, LowerLinearLimit, UpperLinearLimit,
                         LowerAngleLimit, UpperAngleLimit);
      robot_models_[pHinge2->GetName()] = pHinge2;
      LOG(debug_level_) << "SUCCESS: Built Hinge2 between "
                        << sParentName << " and " << sChildName;
    } else if (sJointType == "PToPJoint") {
      //////////////
      //  Point to Point (PToP)
      //////////////
      std::string sParentName;
      std::string sChildName;
      std::vector<double> pivot_in_A;
      std::vector<double> pivot_in_B;
      Eigen::Vector3d eig_pivot_A;
      Eigen::Vector3d eig_pivot_B;
      tinyxml2::XMLElement *pChild = pElement->FirstChildElement();
      //  Construct joint based on children information.
      //  This information may include links, origin, axis.etc
      while (pChild) {
        const char * sName = pChild->Name();
        //  get parent link of joint
        if (strcmp(sName, "parent") == 0) {
          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        //  get child link of joint
        if (strcmp(sName, "child") == 0) {
          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        if (strcmp(sName, "pivot in A") == 0) {
          pivot_in_A = GenNumFromChar(pChild->Attribute("setting"));
          eig_pivot_A << pivot_in_A[0], pivot_in_A[1], pivot_in_A[2];
        }
        if (strcmp(sName, "pivot in B") == 0) {
          pivot_in_B = GenNumFromChar(pChild->Attribute("axis1"));
          eig_pivot_B << pivot_in_B[0], pivot_in_B[1], pivot_in_B[2];
        }
        pChild = pChild->NextSiblingElement();
      }
      //  If there are two shapes, then they are connected
      //  If there is no child, it means the constraint is connected
      //  to the World
      if (sChildName.length() == 0) {
        std::shared_ptr<PToPOne> pPToP = std::make_shared<PToPOne>(
            sJointName,
            static_cast<Shape*>(robot_models_.find(sParentName)->second.get()),
            eig_pivot_A);
        robot_models_[pPToP->GetName()] = pPToP;
      } else {
        std::shared_ptr<PToPTwo> pPToP = std::make_shared<PToPTwo>(
            sJointName,
            static_cast<Shape*>(robot_models_.find(sParentName)->second.get()),
            static_cast<Shape*>(robot_models_.find(sChildName)->second.get()),
            eig_pivot_A, eig_pivot_B);
        robot_models_[pPToP->GetName()] = pPToP;
      }
    }

    /////////////////////////
    // TODO(anyone): Six Degrees of Freedom
    /////////////////////////
    //   else if (sJointType == "SixDOFJoint") {
    //     std::string sParentName;
    //     std::string sChildName;
    //     std::vector<double> transform_in_A;
    //     std::vector<double> transform_in_B;
    //     Eigen::Vector6d eig_transform_A;
    //     Eigen::Vector6d eig_transform_B;
    //     std::vector<double> vAnchor;
    //     std::vector<double> vAxis1;
    //     std::vector<double> vAxis2;
    //     std::vector<double> vLowerLinearLimit;
    //     std::vector<double> vUpperLinearLimit;
    //     std::vector<double> vLowerAngleLimit;
    //     std::vector<double> vUpperAngleLimit;
    //     Eigen::Vector3d Anchor;
    //     Eigen::Vector3d Axis1;
    //     Eigen::Vector3d Axis2;
    //     Eigen::Vector3d LowerLinearLimit;
    //     Eigen::Vector3d UpperLinearLimit;
    //     Eigen::Vector3d LowerAngleLimit;
    //     Eigen::Vector3d UpperAngleLimit;
    //     tinyxml2::XMLElement *pChild = pElement->FirstChildElement();
    //     //  Construct joint based on children information.
    //     while (pChild) {
    //       const char * sName = pChild->Name();
    //       //  get parent link of joint
    //       if (strcmp(sName, "parent") == 0) {
    //         sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
    //       }
    //       //  get child link of joint
    //       if (strcmp(sName, "child") == 0) {
    //         sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
    //       }
    //       if (strcmp(sName, "pivot in A") == 0) {
    //         pivot_in_A = GenNumFromChar(pChild->Attribute("setting"));
    //         eig_pivot_A << pivot_in_A[0], pivot_in_A[1], pivot_in_A[2];
    //       }
    //       if (strcmp(sName, "pivot in B") == 0) {
    //         pivot_in_B = GenNumFromChar(pChild->Attribute("axis1"));
    //         eig_pivot_B = pivot_in_B[0], pivot_in_B[1], pivot_in_B[2];
    //       }
    //       pChild = pChild->NextSiblingElement();
    //     }
    //   }
  }
}


////////////////////////////////////////////////////////////
/// Parse Raycast Car
////////////////////////////////////////////////////////////
SimRaycastVehicle* URDFParser::ParseRaycastCar(std::string sRobotName,
                                               tinyxml2::XMLElement *pElement) {
  LOG(debug_level_) << "Building a RaycastVehicle";
  std::vector<double> vParameters;
  vParameters.resize(29);
  std::vector<double> pose;
  std::string body_mesh = "NONE";
  std::string wheel_mesh = "NONE";
  std::vector<double> body_dim;
  std::vector<double> wheel_dim;

  tinyxml2::XMLElement *pChild = pElement->FirstChildElement();

  while (pChild) {
    std::string sAttrName = pChild->Name();

    //  Car paramters
    //  All of these are stored in a std::vector of doubles. Access them through
    //  their enum specification in ModelGraph/VehicleEnums.h

    if (!sAttrName.compare("param")) {
      std::string param = pChild->Attribute("name");
      if (!param.compare("control delay")) {
        vParameters[6] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("stiffness")) {
        vParameters[12] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("susp conn height")) {
        vParameters[11] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("max susp force")) {
        vParameters[13] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("damp factor")) {
        vParameters[16] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("exp damp factor")) {
        vParameters[17] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("roll influence")) {
        vParameters[18] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("steering coeff")) {
        vParameters[19] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("max steering")) {
        vParameters[20] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("max steering rate")) {
        vParameters[21] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("accel offset")) {
        vParameters[22] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("steering offset")) {
        vParameters[23] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("stall torque coeff")) {
        vParameters[24] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("torque speed slope")) {
        vParameters[25] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("susp rest length")) {
        vParameters[15] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("max susp travel")) {
        vParameters[14] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("Magic B")) {
        vParameters[26] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("Magic C")) {
        vParameters[27] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!param.compare("Magic E")) {
        vParameters[28] = GenNumFromChar(pChild->Attribute("value")).front();
      }
    } else if (!sAttrName.compare("body")) {
      //  Vehicle body parameters
      std::string body = pChild->Attribute("name");
      if (!body.compare("width")) {
        vParameters[0] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!body.compare("length")) {
        vParameters[1] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!body.compare("height")) {
        vParameters[2] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!body.compare("mass")) {
        vParameters[7] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!body.compare("pose")) {
        pose = GenNumFromChar(pChild->Attribute("value"));
      }
      if (!body.compare("mesh")) {
        body_mesh = pChild->Attribute("path");
        body_dim = GenNumFromChar(pChild->Attribute("dim"));
      }
    } else if (!sAttrName.compare("wheel")) {
      //  Vehicle wheel parameters
      std::string wheel = pChild->Attribute("name");
      if (!wheel.compare("radius")) {
        vParameters[8] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!wheel.compare("width")) {
        vParameters[9] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!wheel.compare("dyn friction")) {
        vParameters[3] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!wheel.compare("slip coeff")) {
        vParameters[5] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!wheel.compare("traction friction")) {
        vParameters[10] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!wheel.compare("side friction")) {
        vParameters[4] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if (!wheel.compare("mesh")) {
        wheel_mesh = pChild->Attribute("path");
        LOG(debug_level_) << "Path to Vehicle mesh is " << wheel_mesh;
        wheel_dim = GenNumFromChar(pChild->Attribute("dim"));
      }
    }

    pChild = pChild->NextSiblingElement();
  }

  Eigen::Vector6d dPose;
  dPose << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
  std::shared_ptr<SimRaycastVehicle> pRaycastVehicle =
      std::make_shared<SimRaycastVehicle>(sRobotName,
                                          vParameters,
                                          dPose);
  if (body_mesh!= "NONE" && wheel_mesh!= "NONE") {
    pRaycastVehicle->SetMeshes(body_mesh, wheel_mesh, body_dim, wheel_dim);
  }

  // Build the car here.
  robot_models_[sRobotName] = pRaycastVehicle;
  LOG(debug_level_) << "SUCCESS: Built RaycastVehicle " << sRobotName;
  return pRaycastVehicle.get();
}


//////////////////////////////////////////////////////////////
/// Parse SENSOR BODIES. Automatically Create Body for Sensor
//////////////////////////////////////////////////////////////
void URDFParser::ParseSensorShape(std::string sRobotName,
                                  tinyxml2::XMLElement *pElement) {
  const char* sRootContent = pElement->Name();
  if (strcmp(sRootContent, "Device") == 0) {
    std::string sParentName = pElement->Attribute("Parent");
    //  This means that it's a controller, not a sensor.
    if (sParentName == "NONE") {
      return;
    }
    LOG(debug_level_) << "Building a sensor body";
    std::string sCameraName = GetAttribute(pElement, "Name")+"@"+sRobotName;
    sParentName = GetAttribute(pElement, "Parent")+"@"+sRobotName;
    std::vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));
    std::vector<double> vMass = GenNumFromChar(pElement->Attribute("Mass"));
    std::vector<double> vDimension =
        GenNumFromChar(pElement->Attribute("Dimension"));

    //  CREATE THE PHYSICS BODY

    Shape* parent = static_cast<Shape*>(
        robot_models_.find(sParentName)->second.get());
    Eigen::Vector6d parent_pose = parent->GetPose();
    std::vector<double> vDepthCameraPose;
    vDepthCameraPose.push_back(vPose[0]+parent_pose(0));
    vDepthCameraPose.push_back(vPose[1]+parent_pose(1));
    vDepthCameraPose.push_back(vPose[2]+parent_pose(2));
    vDepthCameraPose.push_back(vPose[3]+parent_pose(3));
    vDepthCameraPose.push_back(vPose[4]+parent_pose(4));
    vDepthCameraPose.push_back(vPose[5]+parent_pose(5));
    std::shared_ptr<BoxShape> pCameraBox = std::make_shared<BoxShape>(
        sCameraName,
        vDimension[0], vDimension[1],
        vDimension[2], vMass[0], 1,
        vDepthCameraPose);
    robot_models_[pCameraBox->GetName()] = pCameraBox;

    //  CREATE THE PHYSICS CONSTRAINT

    Eigen::Vector3d vPivot;
    Eigen::Vector3d vAxis;
    std::string sCameraJointName = "SimCamJoint"+sCameraName;
    vPivot <<  -vPose[0], -vPose[1], -vPose[2];
    vAxis <<  1, 0, 0;
    std::shared_ptr<HingeTwoPivot> pCameraHinge =
        std::make_shared<HingeTwoPivot>(
            sCameraJointName,
            static_cast<Shape*>(robot_models_.find(sParentName)->second.get()),
            static_cast<Shape*>(robot_models_.find(sCameraName)->second.get()),
            Eigen::Vector3d::Zero(), vPivot, vAxis, vAxis);
    pCameraHinge->SetLimits(-0.01, 0.01, 1, .1, 1);
    robot_models_[pCameraHinge->GetName()] = pCameraHinge;
    LOG(debug_level_) << "SUCCESS: Built sensor body";
  }
}

////////////////////////////////////////////////////////////
/// PARSE ROBOT.XML FOR DEVICES AND BUILD INTO LocalSim
////////////////////////////////////////////////////////////
bool URDFParser::ParseDevices(tinyxml2::XMLDocument& pDoc,
                              const std::string sProxyName,
                              std::shared_ptr<SimDevices> sim_devices) {
  tinyxml2::XMLElement *pParent = pDoc.RootElement();
  tinyxml2::XMLElement *pElement = pParent->FirstChildElement();
  std::string sRobotName(GetAttribute(pParent, "name"));
  sRobotName = sProxyName;


  //  read high level parent (root parent)
  while (pElement) {
    const char* sRootContent = pElement->Name();
    if (strcmp(sRootContent, "Device") == 0) {
      std::string sType(pElement->Attribute("Type"));

      ///////////////////////////
      // SENSOR DEVICES
      // Include everything in SimDevices::Sensor directory
      ///////////////////////////

      // CAMERAS
      if (sType == "Camera") {
        LOG(debug_level_) << "Add device: Camera";
        std::string sMode = GetAttribute(pElement, "Mode");
        std::string sModel = GetAttribute(pElement, "Model");
        int iFPS = atoi(GetAttribute(pElement, "FPS").c_str());
        std::string sDeviceName = GetAttribute(pElement, "Name")+"@"+sRobotName;
        std::vector<double> dPose = GenNumFromChar(pElement->Attribute("Pose"));
        Eigen::Vector6d vPose;
        vPose << dPose[0], dPose[1], dPose[2], dPose[3], dPose[4], dPose[5];
        //  Single-view systems: RGB, Grey, Depth
        if (sMode == "RGB") {
          SimCamera* Device =
              new SimCamera(sDeviceName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamRGB, iFPS, vPose, sModel);
          sim_devices->AddDevice(Device);
        }
        if (sMode == "Depth") {
          SimCamera* Device =
              new SimCamera(sDeviceName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamDepth, iFPS, vPose, sModel);
          sim_devices->AddDevice(Device);
        }
        if (sMode == "Grey") {
          SimCamera* Device =
              new SimCamera(sDeviceName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamLuminance, iFPS, vPose, sModel);
          sim_devices->AddDevice(Device);
        }

        //  Double-view system: RGB-Depth Camera
        if (sMode == "RGBD") {
          //  RGB Camera
          std::string sCameraName = "RGB_"+sDeviceName;
          SimCamera* RGBDevice =
              new SimCamera(sCameraName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamRGB, iFPS, vPose, sModel);
          sim_devices->AddDevice(RGBDevice);
          //  Depth Camera
          sCameraName = "Depth_"+sDeviceName;
          SimCamera* DepthDevice =
              new SimCamera(sCameraName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamDepth, iFPS, vPose, sModel);
          sim_devices->AddDevice(DepthDevice);
        }
        LOG(debug_level_) << "SUCCESS: SimCamera initialized.";
        LOG(debug_level_) << "         Camera type: " << sMode;
      }

      // GPS
      if (sType == "GPS") {
        LOG(debug_level_) << "Add device: GPS";
        std::string sDeviceName = GetAttribute(pElement, "Name")+"@"+sRobotName;
        SimGPS* pGPS = new SimGPS(sDeviceName, sDeviceName, sRobotName);
        sim_devices->AddDevice(pGPS);
      }

      // Vicon
      if (sType == "Vicon") {
        LOG(debug_level_) << "Add device: Vicon";
      }

      ///////////////////////////
      // CONTROLLER DEVICES
      // Include everything in SimDevices::Controller directory
      ///////////////////////////

      if (sType == "CarController") {
        LOG(debug_level_) << "Add device: CarController";
        std::string sControllerName = GetAttribute(pElement, "Name")
            + "@" + sRobotName;
        std::string sBodyName = GetAttribute(pElement, "Body")+"@"+sRobotName;
        CarController* CarDevice =
            new CarController(sControllerName, sBodyName, sRobotName);
        sim_devices->AddDevice(CarDevice);
      }

      if (sType == "SimpleController") {
        LOG(debug_level_) << "Add device: SimpleController";
        std::string sControllerName = GetAttribute(pElement, "Name")
            + "@" + sRobotName;
        std::string sBodyName = GetAttribute(pElement, "Body")+"@"+sRobotName;
        SimpleController* SimpleDevice =
            new SimpleController(sControllerName, sBodyName, sRobotName);
        sim_devices->AddDevice(SimpleDevice);
      }
    }
    //  read next parent element
    pElement = pElement->NextSiblingElement();
  }
  return true;
}


////////////////////////////////////////////////////////////////////////////
/// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////

std::vector<std::shared_ptr<ModelNode> > URDFParser::GetModelNodes(
    std::map<std::string, std::shared_ptr<ModelNode> > mNodes) {
  std::vector<std::shared_ptr<ModelNode> > Nodes;
  for (std::map<std::string, std::shared_ptr<ModelNode> >::iterator it =
           mNodes.begin();
       it!= mNodes.end(); it++) {
    Nodes.push_back(it->second);
  }
  return Nodes;
}
