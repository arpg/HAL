#ifndef POSE_H
#define POSE_H

#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include "Messages.pb.h"
#include "Matrix.h"

namespace pb {

template <typename Scalar>
void WritePoseSE3(const Sophus::SE3Group<Scalar>& pose, PoseMsg* msg) {
  msg->set_type(pb::PoseMsg_Type_SE3);
  WriteVector(Sophus::Map<const Eigen::Matrix<Scalar, 7, 1> >(pose.data()),
              msg->mutable_pose());
}

template <typename Scalar>
void ReadPoseSE3(const PoseMsg &msg, Sophus::SE3Group<Scalar>* pose) {
  *pose = Eigen::Map<const Sophus::SE3Group<Scalar> >(msg.pose().data().data());
}

}

#endif // POSE_H
