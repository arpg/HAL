#ifndef POSE_H
#define POSE_H

#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <Messages/Messages.pb.h>
#include "Matrix.h"

namespace pb {

namespace Eigen{
    typedef Eigen::Matrix<double,7,1> Vector7d;
}

/////////////////////////////////////////////////////////////////////////////////////////
void WritePoseSE3(const Sophus::SE3d& pose, PoseMsg &msg)
{
    msg.set_type(pb::PoseMsg_Type_SE3);
    Sophus::Map<Eigen::Matrix<double,7,1> > poseMap(pose.data());
    WriteVector(poseMap,*msg.mutable_pose());
}

/////////////////////////////////////////////////////////////////////////////////////////
Sophus::SE3d ReadPoseSE3(PoseMsg &msg)
{
    Eigen::Vector7d vec = ReadVector(msg.pose());
    Eigen::Map<Sophus::SE3d> pose(vec.data());
    return pose;
}

}

#endif // POSE_H
