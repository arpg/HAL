#ifndef POSE_H
#define POSE_H

#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <HAL/Messages/Matrix.h>

namespace hal {

template <typename Scalar>
void WritePoseSE3(const Sophus::SE3Group<Scalar>& pose, PoseMsg* msg) {
  msg->set_type(hal::PoseMsg_Type_SE3);

  Eigen::Matrix<Scalar, 7, 1> vec(
      Sophus::Map<const Eigen::Matrix<Scalar, 7, 1> >(pose.data()));
  WriteVector(vec.template cast<double>(), msg->mutable_pose());
}

template <typename Scalar>
void ReadPoseSE3(const PoseMsg &msg, Sophus::SE3Group<Scalar>* pose) {
  *pose = Sophus::SE3d(Eigen::Map<const Sophus::SE3d>(
      msg.pose().data().data())).template cast<Scalar>();
}

}

#endif // POSE_H
