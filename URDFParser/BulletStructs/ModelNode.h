// Copyright (c) bminortx

#ifndef URDFPARSER_BULLETSTRUCTS_MODELNODE_H_
#define URDFPARSER_BULLETSTRUCTS_MODELNODE_H_

#include <stdio.h>

#include <URDFParser/EigenHelpers.h>

#include <string>
#include <vector>
#include <map>
#include <memory>

/*********************************************
 * MODEL NODE
 * ModelNode can be thought of as our class of parts. It is further broken down
 * into subclasses of Shapes, Constraints, and RaycastVehicles (a unique body).
 * ModelNodes can have child nodes connected, and each ModelNode has a unique
 * name.
 *********************************************/

class ModelNode {
 public:
  ModelNode() {
    model_id_ = -1;
    model_parent_ = NULL;
    model_children_.clear();
    model_name_ = "ModelNode";
    model_pose_ = Eigen::Vector6d::Identity();
  }

  virtual const char* Name() const {
    return model_name_.c_str();
  }

  // SETTERS
  void SetBase(ModelNode* pBase) {
    pBase->model_parent_.reset(this);
    this->AddChild(pBase);
  }

  void SetName(const std::string& sName) {
    model_name_ = sName;
  }

  // Set the parent of this node
  void SetParent(ModelNode* pParent) {
    model_parent_.reset(pParent);
  }

  void SetPose(double x, double y, double z, double p, double q, double r) {
    model_pose_ << x, y, z, r, p, q;
  }

  void SetPose(std::vector<double> dPose) {
    SetPose(dPose[0], dPose[1], dPose[2],
            dPose[3], dPose[4], dPose[5]);
  }

  // Set pose as (x, y, z, roll, pitch, yaw) vector
  void SetPose(const Eigen::Vector6d& v) {
    SetPose(v(0), v(1), v(2), v(3), v(4), v(5));
  }

  // Set pose as 4x4 matrix
  void SetPose(const Eigen::Matrix4d& Two) {
    model_pose_ = _T2Cart(Two);
  }

  // Set position only (x, y, z)
  void SetPosition(double x, double y, double z) {
    SetPose(x, y, z, model_pose_(3), model_pose_(4), model_pose_(5));
  }

  void SetPosition(std::vector<double> dPosition) {
    SetPosition(dPosition[0], dPosition[1], dPosition[2]);
  }

  // Set position only (x, y, z)
  void SetPosition(Eigen::Vector3d v) {
    SetPosition(v(0), v(1), v(2));
  }



  ///////////////////////////////////////////////////////////////////
  // GETTERS

  std::string GetName(void) {
    return model_name_;
  }

  // Return pose as (x, y, z, roll, pitch, yaw) vector
  Eigen::Vector6d GetPose() const {
    return model_pose_;
  }

  //  Pose (in parent frame)
  Eigen::Matrix4d GetPoseMatrix() const {
    return _Cart2T(model_pose_);
  }

  Eigen::Vector3d GetPositon() {
    return Eigen::Vector3d(model_pose_(0), model_pose_(1), model_pose_(2));
  }

  Eigen::Matrix3d GetRotationMatrix() {
    Eigen::Vector3d rpq;
    rpq << model_pose_(3), model_pose_(4), model_pose_(5);
    return _Cart2R(rpq(0), rpq(1), rpq(2));
  }

  ///////////////////////////////////////////////////////////////////
  //  CHILDREN FUNCTIONS

  void AddChild(ModelNode* pChild) {
    model_children_.push_back(pChild);
  }

  bool RemoveChild(ModelNode* pChild) {
    for (size_t ii = 0; ii < model_children_.size(); ii++) {
      if (model_children_[ii] == pChild) {
        model_children_.erase(model_children_.begin()+ii);
        return true;
      }
    }
    return false;
  }

  size_t NumChildren() const {
    return model_children_.size();
  }

  // Access child bodies
  ModelNode& operator[](int i) {
    return *model_children_[i];
  }

  const ModelNode& operator[](int i) const {
    return *model_children_[i];
  }

  ///////////////////////////////////////////////////////////////////
  // MEMBER VARIABLES

  static std::map<int, ModelNode*> g_mNodes;  // static map of id to node
  static int                       g_nHandleCounter;
  int                              model_id_;
  std::shared_ptr<ModelNode>       model_parent_;
  std::vector<ModelNode*>          model_children_;
  std::string                      model_name_;
  Eigen::Vector6d                  model_pose_;  // object in World frame
  std::string                      model_type_;
};

#endif  // URDFPARSER_BULLETSTRUCTS_MODELNODE_H_
