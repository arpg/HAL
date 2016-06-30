#pragma once

#include <thread>
#include <fstream>

#include <HAL/Posys/PosysDriverInterface.h>

namespace hal {

class CsvPosysDriver : public PosysDriverInterface
{
public:
    CsvPosysDriver(const std::string sFile);
    ~CsvPosysDriver();

    void RegisterPosysDataCallback(PosysDriverDataCallback callback);
    void RegisterPosysFinishedCallback(PosysDriverFinishedCallback callback);
    bool IsRunning() const override {
      return m_bShouldRun;
    }

private:
    void _ThreadCaptureFunc();
    bool _GetNextTime( double& dNextTime, double& dNextTimePPS );

    std::ifstream             m_pFile;
    volatile bool             m_bShouldRun;
    double                    m_dNextTime;
    double                    m_dNextTimePPS;
    std::thread               m_DeviceThread;
    PosysDriverDataCallback   m_PosysCallback;
    PosysDriverFinishedCallback   m_PosysFinishedCallback;
};

} /* namespace */
