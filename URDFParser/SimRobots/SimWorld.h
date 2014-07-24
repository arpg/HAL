// Copyright (c) bminortx

#ifndef URDFPARSER_SIMROBOTS_SIMWORLD_H_
#define URDFPARSER_SIMROBOTS_SIMWORLD_H_

#include <BulletStructs/ModelNode.h>

#include <vector>
#include <string>

struct SimWorld {
  std::vector<double> pose_;
  std::vector<double> normal_;
  std::vector<double> robot_pose_;
  std::vector<std::shared_ptr<ModelNode> > models_;
  std::string mesh_dir_;
};

#endif  // URDFPARSER_SIMROBOTS_SIMWORLD_H_
