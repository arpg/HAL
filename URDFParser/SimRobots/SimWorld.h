// Copyright (c) bminortx

#ifndef URDFPARSER_SIMROBOTS_SIMWORLD_H_
#define URDFPARSER_SIMROBOTS_SIMWORLD_H_

#include <BulletStructs/ModelNode.h>

#include <vector>
#include <string>

// SimWorld is just a struct, really, holding the info we pulled frem the
// world.xml file

class SimWorld {
 public:
  std::vector<double> simworld_pose_;
  std::vector<double> simworld_normal_;
  std::vector<double> simworld_robot_pose_;
  std::vector<ModelNode*> simworld_models_;
  std::string simworld_mesh_dir_;
};

#endif  // URDFPARSER_SIMROBOTS_SIMWORLD_H_
