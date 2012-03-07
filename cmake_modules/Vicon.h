/*
 * File:   Vicon.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef VICON_H
#define	VICON_H


#include <Eigen/Eigen>
#include <boost/thread.hpp>
#include <vrpn_Tracker.h>
#include <vector>
#include "Common.h"


class Vicon : public boost::mutex
{
    public:
        Vicon();
        virtual ~Vicon();
        void TrackObject( const std::string& sObjectName, const std::string& sHost );
        void Start();
        void Stop();
        Eigen::Matrix<double,6,1> GetPose( const std::string& sObjectName );
        eLocType WhereAmI( Eigen::Vector3d P );
        eLocType WhereAmI( Eigen::Vector6d P );

    private:
        static void _ThreadFunction(Vicon *pVT);
        static void VRPN_CALLBACK _MoCapHandler(void* uData, const vrpn_TRACKERCB tData );

    private:

        struct TrackerObject
        {
            Eigen::Vector6d                        m_dSensorPose;
            vrpn_Tracker_Remote*                   m_pTracker;
            Vicon*                                 m_pViconObject;
        };

        std::map< std::string,  TrackerObject >     m_mObjects;

        bool                                        m_bIsStarted;
        boost::thread*								m_pThread;
        vrpn_Connection*                            m_pViconConnection;
};

#endif	/* VICON_H */