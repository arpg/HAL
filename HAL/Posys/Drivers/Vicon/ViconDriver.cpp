#include "ViconDriver.h"

#include <unistd.h>

#include <HAL/Utils/Uri.h>
#include <HAL/Utils/TicToc.h>

using namespace hal;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ViconDriver::ViconDriver(std::string sHost, std::string sTrackedObjs)
    : m_bRunning(false), m_pThread(nullptr)
{
    std::vector<std::string> vTrackedObjs = Expand(sTrackedObjs, '[', ']', ',');

    for( size_t ii = 0; ii < vTrackedObjs.size(); ++ii ) {
        std::string sUri = vTrackedObjs[ii] + "@" + sHost;

        TrackerObject* pObj = &m_mObjects[ vTrackedObjs[ii] ];

        m_pViconConnection = vrpn_get_connection_by_name( sHost.c_str() );

        pObj->m_sName = vTrackedObjs[ii];
        pObj->m_nId = ii;
        pObj->m_pTracker = new vrpn_Tracker_Remote( sUri.c_str(), m_pViconConnection  );
        pObj->m_pViconDriver = this;
        pObj->m_pTracker->register_change_handler( pObj, _ViconHandler );
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ViconDriver::~ViconDriver()
{
    m_bRunning = false;
    if( m_pThread->joinable() ) {
        m_pThread->join();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ViconDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
    m_Callback = callback;
    m_bRunning = true;
    m_pThread = new std::thread(ViconDriver::_ThreadFunction, this);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ViconDriver::_ThreadFunction(ViconDriver *pVT)
{
    while( pVT->m_bRunning ) {

        for( auto it = pVT->m_mObjects.begin(); it != pVT->m_mObjects.end(); it++ ) {
            it->second.m_pTracker->mainloop();
        }

        // small sleep to not eat up all the cpu
        usleep(1000);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VRPN_CALLBACK ViconDriver::_ViconHandler( void* uData, const vrpn_TRACKERCB tData )
{
    TrackerObject* pObj = (TrackerObject*)uData;
    ViconDriver* pVicon = pObj->m_pViconDriver;

    hal::PoseMsg pbMsg;

    const double vicon_timestamp = tData.msg_time.tv_sec + (1e-6 * tData.msg_time.tv_usec);
    pbMsg.set_device_time( vicon_timestamp );
    pbMsg.set_system_time( hal::Tic() );

    pbMsg.set_id( pObj->m_nId );

    pbMsg.set_type( hal::PoseMsg_Type_SE3 );

    hal::VectorMsg* pbVec = pbMsg.mutable_pose();

    pbVec->add_data(tData.pos[0]);
    pbVec->add_data(tData.pos[1]);
    pbVec->add_data(tData.pos[2]);

    pbVec->add_data(tData.quat[0]);
    pbVec->add_data(tData.quat[1]);
    pbVec->add_data(tData.quat[2]);
    pbVec->add_data(tData.quat[3]);

    pVicon->m_Callback(pbMsg);
}


