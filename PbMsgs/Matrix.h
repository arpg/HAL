#pragma once

#include <Eigen/Eigen>
#include <PbMsgs/Messages.pb.h>

namespace pb {


/////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd ReadMatrix(const MatrixMsg &msg)
{
    Eigen::MatrixXd mat;
    mat.resize(msg.rows(),msg.data_size()/msg.rows());
    for(int ii = 0 ; ii < msg.data_size() ; ii++){
        mat(ii) = msg.data(ii);
    }
    return mat;
}

/////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd ReadVector(const VectorMsg &msg)
{
    Eigen::VectorXd vec;
    vec.resize(msg.data_size());
    for(int ii = 0 ; ii < msg.data_size() ; ii++){
        vec(ii) = msg.data(ii);
    }
    return vec;
}

/////////////////////////////////////////////////////////////////////////////////////////
void WriteMatrix(const Eigen::MatrixXd &mat, MatrixMsg &msg)
{
    msg.set_rows(mat.rows());
    msg.mutable_data()->Reserve(mat.rows()*mat.cols());
    for(int ii = 0 ; ii < mat.cols()*mat.rows() ; ii++){
        msg.add_data(mat(ii));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void WriteVector(const Eigen::VectorXd &mat, VectorMsg &msg)
{
    msg.mutable_data()->Reserve(mat.rows());
    for(int ii = 0 ; ii < mat.rows() ; ii++){
        msg.add_data(mat(ii));
    }
}

} /* namespace */
