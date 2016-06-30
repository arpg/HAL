#include <iostream>
#include <cstdlib>

#include <HAL/Utils/TicToc.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Devices/DeviceTime.h>
#include <stdexcept>

#include "CsvPosysDriver.h"

namespace hal
{

///////////////////////////////////////////////////////////////////////////////
CsvPosysDriver::CsvPosysDriver(
        const std::string sFile
        )
{
    m_bShouldRun = false;

    m_dNextTime = 0;
    m_dNextTimePPS = 0;

    // open Posys csv file
    if( sFile.empty() ) {
      throw hal::DeviceException("PosysLog: Csv file is required.");
    } else {
      m_pFile.open( sFile.c_str() );
      if( m_pFile.is_open() == false ) {
        throw hal::DeviceException("PosysLog: Couldn't open file '" + sFile +
                                   "'");
      }
    }

    // read one timestamp.. return false if error occurs
    if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
        throw std::runtime_error("Error obtaining next timestamp");
    }

    // push timestamp to VD queue
    hal::DeviceTime::PushTime( m_dNextTime );
}

///////////////////////////////////////////////////////////////////////////////
CsvPosysDriver::~CsvPosysDriver()
{
    // close capture thread
    m_bShouldRun = false;

//    // this is fugly, but we need to wake up the sleeping capture thread somehow
//    DeviceTime::CONDVAR.notify_all();

    // wait for capture thread to die
    m_DeviceThread.join();
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvPosysDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
    m_PosysCallback = callback;
    if( !m_DeviceThread.joinable() ) {
        // start capture thread
        m_bShouldRun = true;
        m_DeviceThread = std::thread(&CsvPosysDriver::_ThreadCaptureFunc, this);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvPosysDriver::RegisterPosysFinishedCallback(PosysDriverFinishedCallback callback)
{
    m_PosysFinishedCallback = callback;
}

///////////////////////////////////////////////////////////////////////////////
void CsvPosysDriver::_ThreadCaptureFunc()
{
    while( m_bShouldRun ) {
        hal::DeviceTime::WaitForTime( m_dNextTime );

        //---------------------------------------------------------

        // get data and pack
        // we already have the timestamp so no need to read it!
        hal::PoseMsg dataPose;
        std::string sValue;
        getline ( m_pFile, sValue );
        std::vector<std::string> items = hal::Split(sValue, ',');
        // note: timestamps were already read

        dataPose.set_id( atol( items[0].c_str() ) );
        dataPose.set_type(static_cast<hal::PoseMsg_Type>
                          ( atol( items[1].c_str() ) ));

        int N = atol( items[2].c_str() ); // pose data size
        dataPose.mutable_pose()->clear_data();
        for (int i = 0; i < N; ++i) {
          dataPose.mutable_pose()->add_data( atof( items[3+i].c_str() ) );
        }

        const int base = 3 + N;
        N = atol( items[base].c_str() ); // covariance data size
        dataPose.mutable_covariance()->clear_data();
        for (int i = 0; i < N; ++i) {
          dataPose.mutable_covariance()->add_data(
                atof( items[base + 1 + i].c_str() ) );
        }

        dataPose.set_device_time(m_dNextTimePPS);
        dataPose.set_system_time(m_dNextTime);
        if( m_PosysCallback )
          m_PosysCallback( dataPose );

        //---------------------------------------------------------

        // break if EOF
        if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
            // Pop the last measurement so it does not hold up the queue
            hal::DeviceTime::PopTime();
            break;
        }

        // pop front and push next timestamp to queue
        hal::DeviceTime::PopAndPushTime( m_dNextTime );
    }
    m_bShouldRun = false;

    // Notify that the driver stopped running
    if (m_PosysFinishedCallback) {
      m_PosysFinishedCallback();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool CsvPosysDriver::_GetNextTime(
        double& dNextTime,                  //< Output
        double& dNextTimePPS                //< Output
        )
{
    std::string sValue;
    getline ( m_pFile, sValue, ',' ); // system time
    dNextTime = atof( sValue.c_str() );
    getline ( m_pFile, sValue, ',' ); // device time
    dNextTimePPS = atof( sValue.c_str() );

    if( m_pFile.eof() ) {
        return false;
    }

    return true;
}

}
