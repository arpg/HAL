#include "ViconDriver.h"

#include <HAL/Utils/Uri.h>

using namespace hal;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ViconDriver::ViconDriver(std::string sHost, std::string sTrackedObjs)
{
    std::vector<std::string> vTrackedObjs = Expand(sTrackedObjs, '[', ']', ',');

    for( size_t ii = 0; ii < vTrackedObjs.size(); ++ii ) {
        std::string sUri = vTrackedObjs[ii] + "@" + sHost;

        TrackerObject* pObj = &m_mObjects[ vTrackedObjs[ii] ];

        pObj->m_sName = vTrackedObjs[ii];
        pObj->m_pTracker = new vrpn_Tracker_Remote( sUri.c_str(), m_pViconConnection  );
        pObj->m_pViconDriver = this;
        pObj->m_pTracker->register_change_handler( pObj, _ViconHandler );

        m_pViconConnection = vrpn_get_connection_by_name( sHost.c_str() );
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ViconDriver::~ViconDriver()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ViconDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
    m_Callback = callback;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VRPN_CALLBACK ViconDriver::_ViconHandler( void* uData, const vrpn_TRACKERCB tData )
{
    TrackerObject* pObj = (TrackerObject*)uData;
    ViconDriver* pVicon = pObj->m_pViconDriver;

    pb::PoseMsg pbMsg;

    const double vicon_timestamp = tData.msg_time.tv_sec + (1e-6 * tData.msg_time.tv_usec);
    pbMsg.set_device_time( vicon_timestamp );

    pbMsg.set_id( tData.sensor );

    pbMsg.set_type( pb::PoseMsg_Type_SE3 );

    pb::VectorMsg* pbVec = pbMsg.mutable_pose();

    pbVec->add_data(tData.quat[0]);
    pbVec->add_data(tData.quat[1]);
    pbVec->add_data(tData.quat[2]);
    pbVec->add_data(tData.quat[3]);
    pbVec->add_data(tData.pos[0]);
    pbVec->add_data(tData.pos[1]);
    pbVec->add_data(tData.pos[2]);

    pVicon->m_Callback(pbMsg);
}


