#pragma once

#include <Eigen/Eigen>
#include <HAL/Messages.pb.h>

namespace hal {

inline void ReadCommand( const CommanderMsg &msg, int* worldId, double* force, double* curvature, Eigen::Vector3d* vec, double* forceDt, double* phi, bool* noDelay, bool* noUpdate )
{
        *worldId = (int)msg.worldid();
        *force = msg.force();
        *curvature = msg.curvature();

	// Torques
        vec->resize( msg.torques().data_size() );
        for ( int ii = 0; ii < msg.torques().data_size(); ii++ )
                { vec->operator()(ii) = msg.torques().data(ii); }

        *forceDt = msg.forcedt();
        *phi = msg.phi();
        *noDelay = msg.nodelay();
        *noUpdate = msg.noupdate();
}

inline void WriteCommand( const int &worldId, const double &force, const double &curvature, const Eigen::Vector3d &vec, const double &forceDt, const double &phi, const bool &noDelay, const bool &noUpdate, CommanderMsg *msg )
{
        msg->set_worldid(worldId);
        msg->set_force(force);
        msg->set_curvature(curvature);

	// Torques
        msg->mutable_torques()->mutable_data()->Reserve( vec.rows() );
  	for ( int ii = 0; ii < vec.rows(); ii++ )
                { msg->mutable_torques()->add_data(vec(ii)); }

        msg->set_forcedt(forceDt);
        msg->set_phi(phi);
        msg->set_nodelay(noDelay);
        msg->set_noupdate(noUpdate);
}

} // nampespace hal
