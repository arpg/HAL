#ifndef SIMWORLD_H
#define SIMWORLD_H

#include <vector>
#include <string>
#include <iostream>
#include <BulletStructs/ModelNode.h>

using namespace std;

// SimWorld is just a struct, really, holding the info we pulled frem the
// world.xml file

class SimWorld
{
 public:
  vector<double>          m_vWorldPose;
  vector<double>          m_vWorldNormal;
  vector<double>          m_vRobotPose;
  std::vector<ModelNode*> m_WorldNodes;
  std::string             m_sMesh;
};

#endif
