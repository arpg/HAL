#pragma once

#include <Eigen/Eigen>
#include <HAL/Messages.pb.h>

namespace hal {

inline void ReadMatrix(const MatrixMsg &msg, Eigen::MatrixXd* mat) {
  mat->resize(msg.rows(),msg.data_size()/msg.rows());
  for(int ii = 0 ; ii < msg.data_size() ; ii++){
    mat->operator()(ii) = msg.data(ii);
  }
}

inline void ReadVector(const VectorMsg &msg, Eigen::VectorXd* vec) {
  vec->resize(msg.data_size());
  for(int ii = 0 ; ii < msg.data_size() ; ii++){
    vec->operator()(ii) = msg.data(ii);
  }
}

inline void WriteMatrix(const Eigen::MatrixXd &mat, MatrixMsg *msg) {
  msg->set_rows(mat.rows());
  msg->mutable_data()->Reserve(mat.rows()*mat.cols());
  for(int ii = 0 ; ii < mat.cols()*mat.rows() ; ii++){
    msg->add_data(mat(ii));
  }
}

inline void WriteVector(const Eigen::VectorXd &mat, VectorMsg *msg) {
  msg->mutable_data()->Reserve(mat.rows());
  for(int ii = 0 ; ii < mat.rows() ; ii++){
    msg->add_data(mat(ii));
  }
}

}  // namespace hal
