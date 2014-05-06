#pragma once

#include <thread>

#include <vrpn_Tracker.h>

#include <HAL/Posys/PosysDriverInterface.h>

namespace hal {


class ViconDriver : public PosysDriverInterface
{
public:
    ViconDriver(std::string sHost, std::string sTrackedObjs);
    ~ViconDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);

  bool IsRunning() const override {
    return m_bRunning;
  }

private:
    static void _ThreadFunction(ViconDriver *pVT);
    static void VRPN_CALLBACK _ViconHandler(void* uData, const vrpn_TRACKERCB tData);

private:
    PosysDriverDataCallback                     m_Callback;

    struct TrackerObject
    {
        std::string                             m_sName;
        unsigned int                            m_nId;
        vrpn_Tracker_Remote*                    m_pTracker;
        ViconDriver*                            m_pViconDriver;
    };

    std::map< std::string, TrackerObject >      m_mObjects;
    vrpn_Connection*                            m_pViconConnection;

    bool                                        m_bRunning;
    std::thread*                                m_pThread;

};

} /* namespace */
