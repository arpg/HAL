#include "URDFParser/URDFParser.h"

////////////////////////////////////////////////////////////
/// CONSTRUCTOR
////////////////////////////////////////////////////////////

URDFParser::URDFParser(int debug_level){
  debug_level_ = debug_level;
}

////////////////////////////////////////////////////////////
/// PARSE WORLD.XML IN LocalSim
////////////////////////////////////////////////////////////
bool URDFParser::ParseWorld(XMLDocument& pDoc, SimWorld& mSimWorld){
  XMLElement *pParent=pDoc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();

  // read high level parent (root parent)
  while (pElement){
    const char* sRootContent = pElement->Name();
    if(strcmp(sRootContent,"base")==0){
      string sMesh(pElement->Attribute("mesh"));
      mSimWorld.m_sMesh = sMesh;
      mSimWorld.m_vWorldPose =
          GenNumFromChar(pElement->Attribute("worldpose"));
      mSimWorld.m_vWorldNormal =
          GenNumFromChar(pElement->Attribute("worldnormal"));
      mSimWorld.m_vRobotPose=
          GenNumFromChar(pElement->Attribute("robotpose"));
      std::vector<double> vLightPose =
          GenNumFromChar(pElement->Attribute("lightpose"));
      LightShape* pLight = new LightShape("Light", vLightPose);
      m_mWorldNodes[pLight->GetName()] = pLight;
      //** There are several options for mesh design: **//
      // 1. Init world without mesh. Creates a flat plane.
      if (mSimWorld.m_sMesh =="NONE"){
        LOG(debug_level_) << "World's mesh type: " << mSimWorld.m_sMesh;
        // We can't just use a giant box here; the RaycastVehicle won't connect,
        // and will go straight through.
        PlaneShape* pGround = new PlaneShape("Ground", mSimWorld.m_vWorldNormal,
                                             mSimWorld.m_vWorldPose);
        m_mWorldNodes[pGround->GetName()] = pGround;
      }
      // 2. Init world from MATLAB height data.
      else if ( mSimWorld.m_sMesh == "MATLAB") {
        LOG(debug_level_) << "World's mesh type: " << mSimWorld.m_sMesh;
        node_.init("URDF");
        while(!node_.subscribe("MATLAB/Heightmap")){
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        LOG(debug_level_) << "SUCCESS: Subscribed to MATLAB/Heightmap";
        pb::PlannerHeightmapMsg params;
        while(!node_.receive("MATLAB/Heightmap", params)){
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        LOG(debug_level_) << "SUCCESS: Received heightmap data from MATLAB";
        int row_count = params.row_count();
        int col_count = params.col_count();
        std::vector<double> X, Y, Z;
        X.resize(row_count*col_count);
        Y.resize(row_count*col_count);
        Z.resize(row_count*col_count);
        for (int ii=0; ii < params.x_data().size(); ii++) {
          X.at(ii) = params.x_data().Get(ii);
          Y.at(ii) = params.y_data().Get(ii);
          Z.at(ii) = params.z_data().Get(ii);
        }
        HeightmapShape* map_shape = new HeightmapShape("Map",
                                                       row_count, col_count,
                                                       X, Y, Z);
        m_mWorldNodes[map_shape->GetName()] = map_shape;
      }
      // 3. Fill the .xml with all of the values for the mesh file. That way, we
      // won't have to keep importing them from MATLAB every time, which really
      // takes forever.
      else if ( mSimWorld.m_sMesh == "CSV") {
        LOG(debug_level_) << "World's mesh type: " << mSimWorld.m_sMesh;
        double row_count = ::atof(pElement->Attribute("row_count"));
        double col_count = ::atof(pElement->Attribute("col_count"));
        vector<double> x_data = GenNumFromChar(
            pElement->Attribute("x_data"));
        vector<double> y_data = GenNumFromChar(
            pElement->Attribute("y_data"));
        vector<double> z_data = GenNumFromChar(
            pElement->Attribute("z_data"));
        std::vector<double> X, Y, Z;
        X.resize(row_count*col_count);
        Y.resize(row_count*col_count);
        Z.resize(row_count*col_count);
        for (int ii=0; ii < x_data.size(); ii++) {
          X.at(ii) = x_data.at(ii);
          Y.at(ii) = y_data.at(ii);
          Z.at(ii) = z_data.at(ii);
        }
        HeightmapShape* map_shape = new HeightmapShape("Map",
                                                       row_count, col_count,
                                                       X, Y, Z);
        m_mWorldNodes[map_shape->GetName()] = map_shape;
      } else {
        // 4. We actually have a mesh.
        LOG(debug_level_) << "World's mesh type: mesh";
        LOG(debug_level_) << "Mesh directory: " << mSimWorld.m_sMesh;
        MeshShape* pMesh = new MeshShape("Map", mSimWorld.m_sMesh,
                                         mSimWorld.m_vWorldPose);
        m_mWorldNodes[pMesh->GetName()] = pMesh;
      }
    }
    pElement=pElement->NextSiblingElement();
  }
  LOG(debug_level_) << "SUCCESS: World initialized";
  mSimWorld.m_WorldNodes = GetModelNodes(m_mWorldNodes);
  return true;
}

// This function serves as a shortcut; maybe we don't want to
// have everything, you know? Just the data.
void URDFParser::GetMeshData(XMLDocument& pDoc,
                             std::shared_ptr<HeightmapShape>& map_shape){
  XMLElement *pParent=pDoc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();
  // read high level parent (root parent)
  while (pElement){
    const char* sRootContent = pElement->Name();
    if(strcmp(sRootContent,"base")==0){
      string sMesh(pElement->Attribute("mesh"));
      if ( sMesh == "CSV") {
        double row_count = ::atof(pElement->Attribute("row_count"));
        double col_count = ::atof(pElement->Attribute("col_count"));
        vector<double> x_data = GenNumFromChar(
            pElement->Attribute("x_data"));
        vector<double> y_data = GenNumFromChar(
            pElement->Attribute("y_data"));
        vector<double> z_data = GenNumFromChar(
            pElement->Attribute("z_data"));
        map_shape = std::make_shared<HeightmapShape>("Map",
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
bool URDFParser::ParseRobot(XMLDocument& pDoc,
                             SimRobot& rSimRobot,
                             string sProxyName){
  LOG(debug_level_) << "Parsing robot body:";
  XMLElement *pParent=pDoc.RootElement();
  string sRobotName(GetAttribute(pParent, "name"));
  sRobotName = sProxyName;

  rSimRobot.SetName(sRobotName);
  rSimRobot.SetRobotURDF(pDoc.ToDocument());
  rSimRobot.SetProxyName(sProxyName);
  XMLElement *pElement = pParent->FirstChildElement();

  // Need to do something with m_mModelNodes; right now, nothing happens.
  // Which is silly.
  while (pElement){
    const char* sRootContent = pElement->Name();

    //////////////////////////////////////////
    // THE BODY BASE
    //////////////////////////////////////////
    if(strcmp(sRootContent,"bodybase")==0){
      string sBodyName = GetAttribute( pElement, "name");
      if(sBodyName == "RaycastVehicle"){
        // Assign the raycast vehicle as the bodybase.
        sBodyName = sBodyName+"@"+sRobotName;

        //////////////////////////////////////////
        // Raycast Car
        //////////////////////////////////////////
        SimRaycastVehicle* pVehicle = ParseRaycastCar(sBodyName, pElement);
        rSimRobot.SetBase(pVehicle);
        LOG(debug_level_) << "SUCCESS: built base for "<<sBodyName;
      }
      else{
        sBodyName = sBodyName+"@"+sRobotName;
        int iMass =::atoi( pElement->Attribute("mass"));
        vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
        vector<double> vDimension =
            GenNumFromChar(pElement->Attribute("dimension"));
        const char* sType = pElement->Attribute("type");
        if(strcmp(sType, "Box") ==0){
          BoxShape* pBox =new BoxShape(sBodyName, vDimension[0],vDimension[1],
              vDimension[2],iMass, 1, vPose);
          rSimRobot.SetBase( pBox );
          LOG(debug_level_) << "Successfully built base for "<<sBodyName;
          m_mModelNodes[pBox->GetName()] = pBox;
        }
      }
    }

    //////////////////////////////////////////
    // ALL OF OUR BODIES (SHAPES)
    //////////////////////////////////////////
    // Build shapes connected to the body base.
    ParseShape(sRobotName, pElement);

    //////////////////////////////////////////
    // ALL OF OUR SENSOR BODIES
    //////////////////////////////////////////
    ParseSensorShape(sRobotName, pElement);

    //////////////////////////////////////////
    // ALL OF OUR JOINTS (CONSTRAINTS)
    //////////////////////////////////////////
    ParseJoint(sRobotName, pElement);

    // read next parent element
    pElement=pElement->NextSiblingElement();
  }

  rSimRobot.SetParts(GetModelNodes(m_mModelNodes));

  return true;
}


////////////////////////////////////////////////////////////
/// Read command line and try to init sensor based on command line
/// The input command line looks like:
/// Openni:[Name="LCamera", rgb=1, depth=1]//  Openni:[Name="RCamera", rgb=1]//
////////////////////////////////////////////////////////////
bool URDFParser::ParseCommandLineForPickSensor(string sCommandLine){
  // get scheme:
  return true;
}

// get sceme for init device
vector<string> URDFParser::GetScemeFromString(string sCommandLine){
  vector<string> vSceme;

  // get sceme by looking for "//"
  while (sCommandLine.size()!=0){
    string sSceme = sCommandLine.substr(0, sCommandLine.find_first_of("//")+1) ;
    if(sSceme.find("//")!=string::npos){
      cout<<"[GetScemeFromString] Get Sceme: "<<sSceme<<endl;
      vSceme.push_back(sSceme);
    }
    else{
      if(sCommandLine.size()==0){
        return vSceme;
      }
      else{
        cout<<"[GetScemeFromString] Fatal Error! Invalid command line string "<<
              sCommandLine<<endl;
        exit(-1);
      }
    }
  }
}

////////////////////////////////////////////////////////////
/// Parse Shape (Body)
////////////////////////////////////////////////////////////

void URDFParser::ParseShape(string sRobotName, XMLElement *pElement)
{
  const char* sRootContent = pElement->Name();

  if(strcmp(sRootContent,"body")== 0)
  {
    string sBodyName = GetAttribute( pElement, "name")+"@"+sRobotName;
    LOG(debug_level_) << "Building "<< sBodyName;
    int iMass =::atoi( pElement->Attribute("mass"));
    vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
    vector<double> vDimension =
        GenNumFromChar(pElement->Attribute("dimension"));
    const char* sType = pElement->Attribute("type");

    if(strcmp(sType, "Box") == 0){
      BoxShape* pBox =new BoxShape(sBodyName, vDimension[0], vDimension[1],
                                   vDimension[2], iMass, 1, vPose);
      m_mModelNodes[pBox->GetName()] = pBox;
    }

    else if(strcmp(sType,"Cylinder")== 0){
      CylinderShape* pCylinder =new CylinderShape(sBodyName, vDimension[0],
                                                  vDimension[1], iMass,1,
                                                  vPose);
      m_mModelNodes[pCylinder->GetName()] = pCylinder;
    }

    else if(strcmp(sType, "Sphere") == 0){
      SphereShape* pSphere =new SphereShape(sBodyName, vDimension[0],
                                            iMass, 1, vPose);
      m_mModelNodes[pSphere->GetName()] = pSphere;
    }

    else if(strcmp(sType, "Plane") == 0){
      PlaneShape* pPlane =new PlaneShape(sBodyName, vDimension, vPose);
      m_mModelNodes[pPlane->GetName()] = pPlane;
    }

    else if(strcmp(sType, "Mesh") == 0){
      string file_dir = pElement->Attribute("dir");
      MeshShape* pMesh =new MeshShape(sBodyName, file_dir, vPose);
      m_mModelNodes[pMesh->GetName()] = pMesh;
    }

    LOG(debug_level_) << "SUCCESS: Built " << sBodyName;
  }
}


////////////////////////////////////////////////////////////
/// Parse Joint
////////////////////////////////////////////////////////////

void URDFParser::ParseJoint(string sRobotName, XMLElement *pElement){
  const char* sRootContent = pElement->Name();
  if(strcmp(sRootContent,"joint")==0){

    //Attributes common to all joints
    string sJointName= GetAttribute( pElement, "name")+"@"+sRobotName;
    string sJointType(pElement->Attribute("type"));

    ////////////////
    // Hinge
    ////////////////
    if(sJointType == "HingeJoint"){
      LOG(debug_level_) << "Building Hinge";
      string sParentName;
      string sChildName;
      vector<double> vPivot;
      vector<double> vAxis;
      double dUpperLimit = M_PI;
      double dLowerLimit = 0;
      double dSoftness = .5;
      double dBias = .5;
      double dRelaxation = .5;

      // Construct joint based on children information.
      // This information may include links, origin, axis.etc
      XMLElement *pChild=pElement->FirstChildElement();
      while(pChild){
        const char * sName = pChild->Name();
        // get parent body of joint
        if(strcmp(sName, "parent")==0){
          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        // get child body of joint
        if(strcmp(sName, "child")==0){
          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        if(strcmp(sName, "pivot")==0){
          vPivot=GenNumFromChar( pChild->Attribute("setting"));
        }
        if(strcmp(sName, "axis")==0){
          vAxis=GenNumFromChar( pChild->Attribute("setting"));
        }
        if(strcmp(sName,"upperlimit")==0){
          dUpperLimit = ::atof(sName);
        }
        if(strcmp(sName,"lowerlimit")==0){
          dLowerLimit = ::atof(sName);
        }
        if(strcmp(sName,"softness")==0){
          dSoftness = ::atof(sName);
        }
        if(strcmp(sName,"bias")==0){
          dBias = ::atof(sName);
        }
        if(strcmp(sName,"relaxation")==0){
          dRelaxation = ::atof(sName);
        }
        // read next child (joint)
        pChild=pChild->NextSiblingElement();
      }
      Eigen::Vector3d pivot;
      pivot<<vPivot[0], vPivot[1], vPivot[2];
      Eigen::Vector3d axis;
      axis<<vAxis[0], vAxis[1], vAxis[2];
      HingeTwoPivot* pHinge = new HingeTwoPivot(
            sJointName,
            dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
            dynamic_cast<Shape*>(m_mModelNodes.find(sChildName)->second),
            pivot, Eigen::Vector3d::Zero(),
            axis, Eigen::Vector3d::Zero());
      pHinge->SetLimits(dLowerLimit, dUpperLimit,
                        dSoftness, dBias, dRelaxation);
      m_mModelNodes[pHinge->GetName()] = pHinge;
      LOG(debug_level_) << "SUCCESS: Built Hinge";
    }

    ///////////////
    // Hinge2
    ///////////////
    else if(sJointType=="Hinge2Joint"){
      LOG(debug_level_) << "Building Hinge2";
      string sParentName;
      string sChildName;
      vector<double> vAnchor;
      vector<double> vAxis1;
      vector<double> vAxis2;
      vector<double> vLowerLinearLimit;
      vector<double> vUpperLinearLimit;
      vector<double> vLowerAngleLimit;
      vector<double> vUpperAngleLimit;

      Eigen::Vector3d Anchor;
      Eigen::Vector3d Axis1;
      Eigen::Vector3d Axis2;
      Eigen::Vector3d LowerLinearLimit;
      Eigen::Vector3d UpperLinearLimit;
      Eigen::Vector3d LowerAngleLimit;
      Eigen::Vector3d UpperAngleLimit;

      // read detail of a joint
      XMLElement *pChild=pElement->FirstChildElement();

      // Construct joint based on children information.
      // This information may include links, origin, axis.etc
      while(pChild){
        const char * sName = pChild->Name();
        // get parent link of joint
        if(strcmp(sName, "parent")==0){
          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        // get child link of joint
        if(strcmp(sName, "child")==0){
          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        if(strcmp(sName, "anchor")==0){
          vAnchor = GenNumFromChar( pChild->Attribute("setting"));
          Anchor<<vAnchor[0],vAnchor[1],vAnchor[2];
        }
        if(strcmp(sName, "axis")==0){
          vAxis1 = GenNumFromChar( pChild->Attribute("axis1"));
          vAxis2 = GenNumFromChar( pChild->Attribute("axis2"));

          Axis1<<vAxis1[0],vAxis1[1],vAxis1[2];
          Axis2<<vAxis2[0],vAxis2[1],vAxis2[2];
        }
        if(strcmp(sName, "limit")==0){
          vLowerLinearLimit = GenNumFromChar( pChild->Attribute("lowerlinear"));
          vUpperLinearLimit = GenNumFromChar( pChild->Attribute("upperlinear"));
          vLowerAngleLimit = GenNumFromChar( pChild->Attribute("lowerangle"));
          vUpperAngleLimit = GenNumFromChar( pChild->Attribute("upperangle"));
          LowerAngleLimit<<vLowerAngleLimit[0],vLowerAngleLimit[1],
              vLowerAngleLimit[2];
          UpperAngleLimit<<vUpperAngleLimit[0],vUpperAngleLimit[1],
              vUpperAngleLimit[2];
          LowerLinearLimit<<vLowerLinearLimit[0],vLowerLinearLimit[1],
              vLowerLinearLimit[2];
          UpperLinearLimit<<vUpperLinearLimit[0],vUpperLinearLimit[1],
              vUpperLinearLimit[2];
        }
        // read next child (joint)
        pChild=pChild->NextSiblingElement();
      }

      Hinge2* pHinge2 = new Hinge2(
            sJointName,
            dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
            dynamic_cast<Shape*>(m_mModelNodes.find(sChildName)->second),
            Anchor, Axis1, Axis2);
      pHinge2->SetLimits(1, 1, LowerLinearLimit, UpperLinearLimit,
                         LowerAngleLimit, UpperAngleLimit);
      m_mModelNodes[pHinge2->GetName()] = pHinge2;
      LOG(debug_level_) << "SUCCESS: Built Hinge2 between "
                        << sParentName << " and " << sChildName;
    }

    ///////////////
    // Point to Point (PToP)
    ///////////////
    else if(sJointType=="PToPJoint"){
      string sParentName;
      string sChildName;
      vector<double> pivot_in_A;
      vector<double> pivot_in_B;
      Eigen::Vector3d eig_pivot_A;
      Eigen::Vector3d eig_pivot_B;
      XMLElement *pChild=pElement->FirstChildElement();
      // Construct joint based on children information.
      // This information may include links, origin, axis.etc
      while(pChild){
        const char * sName = pChild->Name();
        // get parent link of joint
        if(strcmp(sName, "parent")==0){
          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        // get child link of joint
        if(strcmp(sName, "child")==0){
          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
        }
        if(strcmp(sName, "pivot in A")==0){
          pivot_in_A = GenNumFromChar( pChild->Attribute("setting"));
          eig_pivot_A<<pivot_in_A[0], pivot_in_A[1], pivot_in_A[2];
        }
        if(strcmp(sName, "pivot in B")==0){
          pivot_in_B = GenNumFromChar( pChild->Attribute("axis1"));
          eig_pivot_B<<pivot_in_B[0], pivot_in_B[1], pivot_in_B[2];
        }

        pChild=pChild->NextSiblingElement();

      }

      // If there are two shapes, then they are connected
      // If there is no child, it means the constraint is connected to the World
      if(sChildName.length()==0){
        PToPOne* pPToP = new PToPOne(
              sJointName,
              dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
              eig_pivot_A);
        m_mModelNodes[pPToP->GetName()] = pPToP;
      }
      else{
        PToPTwo* pPToP = new PToPTwo(
              sJointName,
              dynamic_cast<Shape*>(m_mModelNodes.find(sParentName)->second),
              dynamic_cast<Shape*>(m_mModelNodes.find(sChildName)->second),
              eig_pivot_A, eig_pivot_B);
        m_mModelNodes[pPToP->GetName()] = pPToP;
      }
    }

    //////////////////////////
    // TODO: Six Degrees of Freedom
    //////////////////////////
    //    else if(sJointType=="SixDOFJoint"){
    //      string sParentName;
    //      string sChildName;
    //      vector<double> transform_in_A;
    //      vector<double> transform_in_B;
    //      Eigen::Vector6d eig_transform_A;
    //      Eigen::Vector6d eig_transform_B;
    //      vector<double> vAnchor;
    //      vector<double> vAxis1;
    //      vector<double> vAxis2;
    //      vector<double> vLowerLinearLimit;
    //      vector<double> vUpperLinearLimit;
    //      vector<double> vLowerAngleLimit;
    //      vector<double> vUpperAngleLimit;
    //      Eigen::Vector3d Anchor;
    //      Eigen::Vector3d Axis1;
    //      Eigen::Vector3d Axis2;
    //      Eigen::Vector3d LowerLinearLimit;
    //      Eigen::Vector3d UpperLinearLimit;
    //      Eigen::Vector3d LowerAngleLimit;
    //      Eigen::Vector3d UpperAngleLimit;
    //      XMLElement *pChild=pElement->FirstChildElement();
    //      // Construct joint based on children information.
    //      while(pChild){
    //        const char * sName = pChild->Name();
    //        // get parent link of joint
    //        if(strcmp(sName, "parent")==0){
    //          sParentName = GetAttribute(pChild, "body") +"@"+sRobotName;
    //        }
    //        // get child link of joint
    //        if(strcmp(sName, "child")==0){
    //          sChildName = GetAttribute(pChild, "body") +"@"+sRobotName;
    //        }
    //        if(strcmp(sName, "pivot in A")==0){
    //          pivot_in_A = GenNumFromChar( pChild->Attribute("setting"));
    //          eig_pivot_A<<pivot_in_A[0], pivot_in_A[1], pivot_in_A[2];
    //        }
    //        if(strcmp(sName, "pivot in B")==0){
    //          pivot_in_B = GenNumFromChar( pChild->Attribute("axis1"));
    //          eig_pivot_B = pivot_in_B[0], pivot_in_B[1], pivot_in_B[2];
    //        }

    //        pChild=pChild->NextSiblingElement();

    //      }

    //    }


  }
}


////////////////////////////////////////////////////////////
/// Parse Raycast Car
////////////////////////////////////////////////////////////
SimRaycastVehicle* URDFParser::ParseRaycastCar(string sRobotName,
                                                XMLElement *pElement){
  LOG(debug_level_) << "Building a RaycastVehicle";
  std::vector<double> vParameters;
  vParameters.resize(29);
  std::vector<double> pose;
  std::string body_mesh = "NONE";
  std::string wheel_mesh = "NONE";
  std::vector<double> body_dim;
  std::vector<double> wheel_dim;

  XMLElement *pChild = pElement->FirstChildElement();

  while(pChild){
    string sAttrName = pChild->Name();

    // Car paramters
    // All of these are stored in a vector of doubles. Access them through
    // their enum specification in ModelGraph/VehicleEnums.h

    if(!sAttrName.compare("param")){
      std::string param = pChild->Attribute("name");
      if(!param.compare("control delay")){
        vParameters[6] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("stiffness")){
        vParameters[12] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("susp conn height")){
        vParameters[11] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("max susp force")){
        vParameters[13] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("damp factor")){
        vParameters[16] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("exp damp factor")){
        vParameters[17] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("roll influence")){
        vParameters[18] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("steering coeff")){
        vParameters[19] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("max steering")){
        vParameters[20] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("max steering rate")){
        vParameters[21] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("accel offset")){
        vParameters[22] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("steering offset")){
        vParameters[23] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("stall torque coeff")){
        vParameters[24] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("torque speed slope")){
        vParameters[25] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("susp rest length")){
        vParameters[15] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("max susp travel")){
        vParameters[14] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("Magic B")){
        vParameters[26] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("Magic C")){
        vParameters[27] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!param.compare("Magic E")){
        vParameters[28] = GenNumFromChar(pChild->Attribute("value")).front();
      }
    }

    // Vehicle body parameters
    else if(!sAttrName.compare("body")){
      std::string body = pChild->Attribute("name");
      // TODO: Switch length and width throughout
      // right now, one is the other.
      if(!body.compare("width")){
        vParameters[0] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!body.compare("length")){
        vParameters[1] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!body.compare("height")){
        vParameters[2] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!body.compare("mass")){
        vParameters[7] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!body.compare("pose")){
        pose = GenNumFromChar(pChild->Attribute("value"));
      }
      if(!body.compare("mesh")){
        body_mesh = pChild->Attribute("path");
        body_dim = GenNumFromChar(pChild->Attribute("dim"));
      }

    }

    // Vehicle wheel parameters
    else if(!sAttrName.compare("wheel")){
      std::string wheel = pChild->Attribute("name");
      if(!wheel.compare("radius")){
        vParameters[8] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!wheel.compare("width")){
        vParameters[9] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!wheel.compare("dyn friction")){
        vParameters[3] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!wheel.compare("slip coeff")){
        vParameters[5] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!wheel.compare("traction friction")){
        vParameters[10] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!wheel.compare("side friction")){
        vParameters[4] = GenNumFromChar(pChild->Attribute("value")).front();
      }
      if(!wheel.compare("mesh")){
        wheel_mesh = pChild->Attribute("path");
        LOG(debug_level_) << "Path to Vehicle mesh is " << wheel_mesh;
        wheel_dim = GenNumFromChar(pChild->Attribute("dim"));
      }
    }

    pChild=pChild->NextSiblingElement();
  }

  Eigen::Vector6d dPose;
  dPose<<pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];
  SimRaycastVehicle* pRaycastVehicle = new SimRaycastVehicle(sRobotName,
                                                             vParameters,
                                                             dPose);
  if(body_mesh!="NONE" && wheel_mesh!="NONE"){
    pRaycastVehicle->SetMeshes(body_mesh, wheel_mesh, body_dim, wheel_dim);
  }

  /// Build the car here.
  m_mModelNodes[sRobotName] = pRaycastVehicle;
  LOG(debug_level_) << "SUCCESS: Built RaycastVehicle " << sRobotName;
  return pRaycastVehicle;


}


//////////////////////////////////////////////////////////////
/// Parse SENSOR BODIES. Automatically Create Body for Sensor
//////////////////////////////////////////////////////////////
void URDFParser::ParseSensorShape(string sRobotName, XMLElement *pElement ){
  const char* sRootContent = pElement->Name();
  if(strcmp(sRootContent,"Device")==0){
    string sParentName = pElement->Attribute("Parent");
    // This means that it's a controller, not a sensor.
    if(sParentName=="NONE"){
      return;
    }
    LOG(debug_level_) << "Building a sensor body";
    string sCameraName = GetAttribute( pElement, "Name")+"@"+sRobotName;
    sParentName = GetAttribute( pElement, "Parent")+"@"+sRobotName;
    vector<double> vPose = GenNumFromChar(pElement->Attribute("Pose"));
    vector<double> vMass = GenNumFromChar(pElement->Attribute("Mass"));
    vector<double> vDimension =
        GenNumFromChar(pElement->Attribute("Dimension"));

    // CREATE THE PHYSICS BODY

    Shape* parent = dynamic_cast<Shape*>(
          m_mModelNodes.find(sParentName)->second);
    Eigen::Vector6d parent_pose = parent->GetPose();
    vector<double> vDepthCameraPose;
    vDepthCameraPose.push_back(vPose[0]+parent_pose(0));
    vDepthCameraPose.push_back(vPose[1]+parent_pose(1));
    vDepthCameraPose.push_back(vPose[2]+parent_pose(2));
    vDepthCameraPose.push_back(vPose[3]+parent_pose(3));
    vDepthCameraPose.push_back(vPose[4]+parent_pose(4));
    vDepthCameraPose.push_back(vPose[5]+parent_pose(5));
    BoxShape* pCameraBox = new BoxShape(sCameraName,
                                        vDimension[0], vDimension[1],
        vDimension[2], vMass[0], 1,
        vDepthCameraPose);
    m_mModelNodes[pCameraBox->GetName()] = pCameraBox;

    // CREATE THE PHYSICS CONSTRAINT

    Eigen::Vector3d vPivot;
    Eigen::Vector3d vAxis;
    string sCameraJointName = "SimCamJoint"+sCameraName;
    vPivot<< -vPose[0], -vPose[1], -vPose[2];
    vAxis<< 1, 0, 0;
    HingeTwoPivot* pCameraHinge =
        new HingeTwoPivot(sCameraJointName,
                          dynamic_cast<Shape*>(m_mModelNodes.
                                               find(sParentName)->second),
                          dynamic_cast<Shape*>(m_mModelNodes.
                                               find(sCameraName)->second),
                          Eigen::Vector3d::Zero(), vPivot,
                          vAxis, vAxis);
    pCameraHinge->SetLimits(-0.01, 0.01, 1, .1, 1);
    m_mModelNodes[pCameraHinge->GetName()] = pCameraHinge;
    LOG(debug_level_) << "SUCCESS: Built sensor body";
  }
}

////////////////////////////////////////////////////////////
/// PARSE ROBOT.XML FOR DEVICES AND BUILD INTO LocalSim
////////////////////////////////////////////////////////////
bool URDFParser::ParseDevices(XMLDocument& rDoc,
                               SimDevices& m_SimDevices,
                               string sProxyName){
  XMLElement *pParent=rDoc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();
  string sRobotName(GetAttribute(pParent,"name"));
  sRobotName = sProxyName;


  // read high level parent (root parent)
  while (pElement){
    const char* sRootContent = pElement->Name();
    if(strcmp(sRootContent,"Device")==0){
      string sType( pElement->Attribute("Type"));

      ////////////////////////////
      /// SENSOR DEVICES
      /// Include everything in SimDevices::Sensor directory
      ////////////////////////////

      /// CAMERAS
      if(sType == "Camera"){
        LOG(debug_level_) << "Add device: Camera";
        string sMode = GetAttribute(pElement, "Mode");
        string sModel = GetAttribute(pElement, "Model");
        int iFPS = atoi( GetAttribute(pElement,"FPS").c_str());
        string sDeviceName= GetAttribute( pElement, "Name")+"@"+sRobotName;
        vector<double> dPose = GenNumFromChar(pElement->Attribute("Pose"));
        Eigen::Vector6d vPose;
        vPose<<dPose[0], dPose[1], dPose[2], dPose[3], dPose[4], dPose[5];
        // Single-view systems: RGB, Grey, Depth
        if(sMode=="RGB"){
          SimCamera* Device =
              new SimCamera(sDeviceName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamRGB, iFPS, vPose, sModel);
          m_SimDevices.AddDevice(Device);
        }
        if(sMode=="Depth"){
          SimCamera* Device =
              new SimCamera(sDeviceName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamDepth, iFPS, vPose, sModel);
          m_SimDevices.AddDevice(Device);
        }
        if(sMode=="Grey"){
          SimCamera* Device =
              new SimCamera(sDeviceName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamLuminance, iFPS, vPose, sModel);
          m_SimDevices.AddDevice(Device);
        }

        // Double-view system: RGB-Depth Camera
        if(sMode=="RGBD"){
          // RGB Camera
          string sCameraName = "RGB_"+sDeviceName;
          SimCamera* RGBDevice =
              new SimCamera(sCameraName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamRGB, iFPS, vPose, sModel);
          m_SimDevices.AddDevice(RGBDevice);
          // Depth Camera
          sCameraName = "Depth_"+sDeviceName;
          SimCamera* DepthDevice =
              new SimCamera(sCameraName, sDeviceName, sRobotName,
                            SceneGraph::eSimCamDepth, iFPS, vPose, sModel);
          m_SimDevices.AddDevice(DepthDevice);
        }
        LOG(debug_level_) << "SUCCESS: SimCamera initialized.";
        LOG(debug_level_) << "         Camera type: " << sMode;
      }

      /// GPS
      if(sType=="GPS"){
        LOG(debug_level_) << "Add device: GPS";
        string sDeviceName = GetAttribute(pElement, "Name")+"@"+sRobotName;
        SimGPS* pGPS = new SimGPS(sDeviceName, sDeviceName, sRobotName);
        m_SimDevices.AddDevice(pGPS);
      }

      /// Vicon
      if(sType=="Vicon"){
        LOG(debug_level_) << "Add device: Vicon";
        //        string sViconName = GetAttribute( pElement, "Name")+"@"+sRobotName;
        //        SimDeviceInfo* Device = new SimDeviceInfo();
        //        Device->m_sDeviceName = sViconName;
        //        Device->m_sDeviceType = sType;
        //        Device->m_sBodyName = sBodyName;
        //        m_SimDevices.AddDevice(Device);
        //        cout<<"[Proxy/ParseDevice] Add vicon device "<<sViconName<<
        //              " success."<<endl;
      }


      ////////////////////////////
      /// CONTROLLER DEVICES
      /// Include everything in SimDevices::Controller directory
      ////////////////////////////

      if(sType=="CarController"){
        LOG(debug_level_) << "Add device: CarController";
        string sControllerName = GetAttribute(pElement, "Name")+"@"+sRobotName;
        string sBodyName= GetAttribute( pElement, "Body")+"@"+sRobotName;
        CarController* CarDevice =
            new CarController(sControllerName, sBodyName, sRobotName);
        m_SimDevices.AddDevice(CarDevice);
      }

      if(sType=="SimpleController"){
        LOG(debug_level_) << "Add device: SimpleController";
        string sControllerName = GetAttribute(pElement, "Name")+"@"+sRobotName;
        string sBodyName= GetAttribute( pElement, "Body")+"@"+sRobotName;
        SimpleController* SimpleDevice =
            new SimpleController(sControllerName, sBodyName, sRobotName);
        m_SimDevices.AddDevice(SimpleDevice);
      }
    }
    // read next parent element
    pElement=pElement->NextSiblingElement();
  }
  return true;
}


////////////////////////////////////////////////////////////////////////////
/// PARSE WORLD.XML FOR STATEKEEPER
////////////////////////////////////////////////////////////////////////////
bool URDFParser::ParseWorldForInitRobotPose(
    const char* filename,
    vector<Eigen::Vector6d>& rvRobotInitPose){

  // make sure the vector is empty
  rvRobotInitPose.clear();

  // open xml document
  XMLDocument doc;
  if(doc.LoadFile(filename) !=0){
    printf("Cannot open %s\n", filename);
    return false;
  }

  XMLElement *pParent=doc.RootElement();
  XMLElement *pElement=pParent->FirstChildElement();

  // read high level parent (root parent)
  while (pElement){
    const char* sRootContent = pElement->Name();
    if(strcmp(sRootContent,"robot")==0){
      vector<double> vPose = GenNumFromChar(pElement->Attribute("pose"));
      Eigen::Vector6d ePose;
      ePose<<vPose[0], vPose[1], vPose[2], vPose[3], vPose[4], vPose[5];
      rvRobotInitPose.push_back(ePose);
    }
    pElement=pElement->NextSiblingElement();
  }

  return true;
}



////////////////////////////////////////////////////////////////////////////
/// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////

std::vector<ModelNode*> URDFParser::GetModelNodes(
    std::map<std::string, ModelNode*> mNodes){
  std::vector<ModelNode*> Nodes;
  for( std::map<string, ModelNode*>::iterator it = mNodes.begin();
       it!=mNodes.end();it++){
    Nodes.push_back(it->second);
  }
  return Nodes;
}
