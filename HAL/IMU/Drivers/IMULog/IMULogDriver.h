#ifndef IMULOGDRIVER_H
#define IMULOGDRIVER_H

#include "HAL/IMU/IMUDriverInterface.h"

#include <fstream>
#include <thread>

class IMULogDriver : public IMUDriverInterface
{

public:
    IMULogDriver();
    ~IMULogDriver();
    bool Init();
    void RegisterDataCallback(IMUDriverDataCallback callback);
    void RegisterDataCallback(GPSDriverDataCallback callback);

private:
    bool _GetNextTime( double& dNextTime, double& dNextTimePPS );
    static void _ThreadCaptureFunc( IMULogDriver* Ptr );

private:
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
    GPSDriverDataCallback   m_GPSCallback;

};

#endif // IMULOGDRIVER_H
