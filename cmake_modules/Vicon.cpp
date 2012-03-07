/*
 * File:   Vicon.cpp
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#include "Vicon.h"

#include <Mvlpp/Mvl.h>
#include <math.h>

#include "Common.h"
#include "MochaException.h"


//////////////////////////////////////////////////////////////////
Eigen::Vector3d Quat2Euler( double *Q )
{
    Eigen::Vector3d X;

    X(0)   = atan2((Q[2] * Q[3] + Q[0] * Q[1]) * 2,
            (Q[1] * Q[1] + Q[2] * Q[2])*-2 + 1) ;

    X(1) = -asin((Q[0] * Q[2] - Q[1] * Q[3])*2);

    X(2)   = atan2((Q[1] * Q[2] + Q[0] * Q[3])*2,
            (Q[2] * Q[2] + Q[3] * Q[3])*-2 + 1 ) ;

    return X;
}

//////////////////////////////////////////////////////////////////
Vicon::Vicon()
{
    m_pViconConnection = NULL;
    m_bIsStarted = false;
}

//////////////////////////////////////////////////////////////////
void Vicon::TrackObject(
        const std::string& sObjectName,
        const std::string& sHost
        )
{

    std::string sUri = sObjectName + "@" + sHost;

    TrackerObject* pObj = &m_mObjects[ sObjectName ];

    pObj->m_pTracker = new vrpn_Tracker_Remote( sUri.c_str(), m_pViconConnection  );
    pObj->m_pViconObject = this;
    pObj->m_pTracker->register_change_handler( pObj, _MoCapHandler );

    m_pViconConnection = vrpn_get_connection_by_name( sHost.c_str() );
}

//////////////////////////////////////////////////////////////////
Vicon::~Vicon()
{
    std::map< std::string, TrackerObject >::iterator it;
    for( it = m_mObjects.begin(); it != m_mObjects.end(); it++ ){
        delete( it->second.m_pTracker );
    }
}

//////////////////////////////////////////////////////////////////
void Vicon::Start()
{
    if( m_bIsStarted == true ) {
        throw Mochaccino::MochaException("The Vicon thread has already started.");
    }

    m_pThread = new boost::thread(Vicon::_ThreadFunction, this);
    m_bIsStarted = true;
}

//////////////////////////////////////////////////////////////////
void Vicon::Stop()
{
    if( m_bIsStarted == false ) {
        throw Mochaccino::MochaException("No thread is running!");
    }

    m_pThread->interrupt();
    m_pThread->join();

    m_bIsStarted = false;
}

//////////////////////////////////////////////////////////////////
//
Eigen::Matrix<double,6,1> Vicon::GetPose( const std::string& sObjectName )
{
    if( m_mObjects.find( sObjectName ) == m_mObjects.end() ){
        throw Mochaccino::MochaException("Invalid object name.");
    }

    //lock the sensor pose for readout
    lock();
    Eigen::Matrix<double,6,1> pose = m_mObjects[sObjectName].m_dSensorPose;
    unlock();

    return pose;
}

//////////////////////////////////////////////////////////////////
eLocType Vicon::WhereAmI( Eigen::Vector3d P )
{

    // get threshold from CVar
    double dZThreshold = g_MochaConfig.m_dZThreshold;

    double dRampWidth = g_MochaConfig.m_dRampWidth;
    double dRampHeight = g_MochaConfig.m_dRampHeight;
    double dRampRadius = g_MochaConfig.m_dRampRadius;

    unsigned int NumRamps = g_MochaConfig.m_nNumRecRamps + g_MochaConfig.m_nNumCirRamps;

    /*
    std::map< std::string, TrackerObject >::iterator it;
    for( it = m_mObjects.begin(); it != m_mObjects.end(); it++ ){
        TrackerObject& obj = *it->second;
        // Convert Ramp's 6-DOF Cart to T matrix
        Eigen::Matrix4d Twr = mvl::Cart2T( obj.m_dSensorPose );

        // We bring the point to the ramp's reference frames
        Eigen::Vector4d Pr;
        Pr << P, 1;
        Pr = mvl::TInv(Twr) * Pr;

        // Check if we are within ramp's "area"
        if ( ii <= g_MochaConfig.m_nNumRecRamps ) {
            // rectangular ramps
            if ( (fabs(Pr(0)) < dRampHeight / 2) && (fabs(Pr(1)) < dRampWidth / 2) && (Pr(2) <= dZThreshold) ) {
                return VT_REC_RAMP;
            }
        } else {
            // circular ramps
            if ( (sqrt(Pr(0) * Pr(0) + Pr(1) * Pr(1)) < dRampRadius) && (Pr(2) <= dZThreshold) ) {
                return VT_CIR_RAMP;
            }
        }
    }
    */

    // if on the floor
    if ( -P(2) < 0 + dZThreshold ) {
        return VT_GROUND;
    }

    // in the air!
    return VT_AIR;
}

//////////////////////////////////////////////////////////////////
eLocType Vicon::WhereAmI( Eigen::Matrix<double, 6, 1 > P )
{
    Eigen::Vector3d Pv = P.block < 3, 1 > (0, 0);
    return WhereAmI( Pv );
}

//////////////////////////////////////////////////////////////////
void Vicon::_ThreadFunction(Vicon *pV)
{
    while (1) {
        std::map< std::string, TrackerObject >::iterator it;
        for( it = pV->m_mObjects.begin(); it != pV->m_mObjects.end(); it++ ) {
            it->second.m_pTracker->mainloop();
        }
    }
}

//////////////////////////////////////////////////////////////////
void VRPN_CALLBACK Vicon::_MoCapHandler( void* uData, const vrpn_TRACKERCB tData )
{
    TrackerObject* pObj = (TrackerObject*)uData;
    Vicon* pVicon = pObj->m_pViconObject;

    //lock the sensor poses as we update them
    pVicon->lock();

    Eigen::Matrix3d T;
    T << -1,0,0,
         0, 1,0,
         0,0,-1;
    Eigen::Vector3d Pos;
    Pos << tData.pos[0], tData.pos[1], tData.pos[2];
    Pos = T * Pos;

    pObj->m_dSensorPose(0) = Pos(0);
    pObj->m_dSensorPose(1) = Pos(1);
    pObj->m_dSensorPose(2) = Pos(2);

    double Quats[4] = { tData.quat[0], tData.quat[1], tData.quat[2], tData.quat[3] };

    pObj->m_dSensorPose.block < 3, 1 > (3, 0) = Quat2Euler( Quats );

    Eigen::Vector3d Rot;
    Rot << pObj->m_dSensorPose(3), pObj->m_dSensorPose(4), pObj->m_dSensorPose(5);
    Rot = T * Rot;

    pObj->m_dSensorPose(3) = Rot(0);
    pObj->m_dSensorPose(4) = Rot(1);
    pObj->m_dSensorPose(5) = Rot(2);

    std::cout << (pObj->m_dSensorPose).transpose() << std::endl;

    pVicon->unlock();
}