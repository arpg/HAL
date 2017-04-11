#pragma once

#include <thread>
#include <fstream>

#include <HAL/IMU/IMUDriverInterface.h>

namespace hal {

class CsvDriver : public IMUDriverInterface
{
public:
    CsvDriver(const std::string sFileAccel, const std::string sFileGyro,
               const std::string sFileMag,
               const std::string sFileTimestamp
               );
    ~CsvDriver();

    void RegisterIMUDataCallback(IMUDriverDataCallback callback);
    void RegisterIMUFinishedCallback(IMUDriverFinishedCallback callback);
    bool IsRunning() const override {
      return m_bShouldRun;
    }

private:
    void _ThreadCaptureFunc();
    bool _GetNextTime( double& dNextTime, double& dNextTimePPS );

    bool                    m_bHaveAccel;
    bool                    m_bHaveGyro;
    bool                    m_bHaveMag;
    bool                    m_bHaveGPS;
    std::string             m_sDataSourceDir;
    std::ifstream           m_pFileTime;
    std::ifstream           m_pFileAccel;
    std::ifstream           m_pFileGyro;
    std::ifstream           m_pFileMag;
    std::ifstream           m_pFileGPS;
    volatile bool           m_bShouldRun;
    double                  m_dNextTime;
    double                  m_dNextTimePPS;
    std::thread             m_DeviceThread;
    IMUDriverDataCallback   m_IMUCallback;
    IMUDriverFinishedCallback m_IMUFinishedCallback;
};

} /* namespace */
