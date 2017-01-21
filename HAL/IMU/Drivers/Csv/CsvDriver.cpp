#include <iostream>

#include <HAL/Utils/TicToc.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Devices/DeviceTime.h>
#include <stdexcept>

#include "CsvDriver.h"

namespace hal
{

///////////////////////////////////////////////////////////////////////////////
CsvDriver::CsvDriver(
        const std::string sFileAccel,
        const std::string sFileGyro,
        const std::string sFileMag,
        const std::string sFileTimestamp
        )
{
    m_bShouldRun = false;
    m_bHaveAccel = false;
    m_bHaveGyro = false;
    m_bHaveMag = false;
    m_bHaveGPS = false;

    m_dNextTime = 0;
    m_dNextTimePPS = 0;

    // open IMU log files
    if( sFileTimestamp.empty() ) {
        throw hal::DeviceException("IMULog: File with timestamps is required.");
    } else {
        m_pFileTime.open( sFileTimestamp.c_str() );
        if( m_pFileTime.is_open() == false ) {
            throw hal::DeviceException("IMULog: Couldn't open required timestamp file '"+sFileTimestamp+"'");
        }
    }

    if( sFileAccel.empty() == false ) {
        m_pFileAccel.open( sFileAccel.c_str() );
        if( m_pFileAccel.is_open() == false ) {
            std::cerr << "IMULog: Couldn't open accel file '" << sFileAccel << "'" << std::endl;
        } else {
            m_bHaveAccel = true;
        }
    }

    if( sFileGyro.empty() == false ) {
        m_pFileGyro.open( sFileGyro.c_str() );
        if( m_pFileGyro.is_open() == false ) {
            std::cerr << "IMULog: Couldn't open gyro file '" << sFileGyro << "'" << std::endl;
        } else {
            m_bHaveGyro = true;
        }
    }

    if( sFileMag.empty() == false ) {
        m_pFileMag.open( sFileMag.c_str() );
        if( m_pFileMag.is_open() == false ) {
            std::cerr << "IMULog: Couldn't open mag file '" << sFileMag << "'" << std::endl;
        } else {
            m_bHaveMag = true;
        }
    }

//    if( sFileGPS.empty() == false ) {
//        m_pFileGPS.open( sFileGPS.c_str() );
//        if( m_pFileTime.is_open() == false ) {
//            throw hal::DeviceException("IMULog: Couldn't open GPS file '"+sFileGPS+"'");
//        } else {
//            m_bHaveGPS = true;
//        }
//    }

    // read one timestamp.. return false if error occurs
    if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
        throw std::runtime_error("Umm..");
    }

    // push timestamp to VD queue
    hal::DeviceTime::PushTime( m_dNextTime );

}

///////////////////////////////////////////////////////////////////////////////
CsvDriver::~CsvDriver()
{
    // close capture thread
    m_bShouldRun = false;

//    // this is fugly, but we need to wake up the sleeping capture thread somehow
//    DeviceTime::CONDVAR.notify_all();

    // wait for capture thread to die
    m_DeviceThread.join();

    // close IMU log files
    if( m_pFileTime.is_open() ) {
        m_pFileTime.close();
    }
    if( m_pFileAccel.is_open() ) {
        m_pFileAccel.close();
    }
    if( m_pFileMag.is_open() ) {
        m_pFileMag.close();
    }
    if( m_pFileGyro.is_open() ) {
        m_pFileGyro.close();
    }
    if( m_pFileGPS.is_open() ) {
        m_pFileGPS.close();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    m_IMUCallback = callback;
    if( !m_DeviceThread.joinable() ) {
        // start capture thread
        m_bShouldRun = true;
        m_DeviceThread = std::thread( &CsvDriver::_ThreadCaptureFunc, this );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvDriver::RegisterIMUFinishedCallback(IMUDriverFinishedCallback callback)
{
    m_IMUFinishedCallback = callback;
}

///////////////////////////////////////////////////////////////////////////////
void CsvDriver::_ThreadCaptureFunc()
{
    while( m_bShouldRun ) {
        hal::DeviceTime::WaitForTime( m_dNextTime );

        //---------------------------------------------------------

        // get data and pack
        // we already have the timestamp so no need to read it!
        std::string     sValue;

        hal::ImuMsg dataIMU;

        if( m_bHaveAccel ) {
            hal::VectorMsg* pbVec = dataIMU.mutable_accel();

            getline ( m_pFileAccel, sValue, ',' );
            pbVec->add_data( atof( sValue.c_str() ) );

            getline ( m_pFileAccel, sValue, ',' );
            pbVec->add_data( atof( sValue.c_str() ) );

            getline ( m_pFileAccel, sValue );
            pbVec->add_data( atof( sValue.c_str() ) );
        }

        if( m_bHaveGyro ) {
            hal::VectorMsg* pbVec = dataIMU.mutable_gyro();

            getline ( m_pFileGyro, sValue, ',' );
            pbVec->add_data(  atof( sValue.c_str() ) );

            getline ( m_pFileGyro, sValue, ',' );
            pbVec->add_data(  atof( sValue.c_str() ) );

            getline ( m_pFileGyro, sValue );
            pbVec->add_data(  atof( sValue.c_str() ) );
        }

        if( m_bHaveMag ) {
            hal::VectorMsg* pbVec = dataIMU.mutable_mag();

            getline ( m_pFileMag, sValue, ',' );
            pbVec->add_data( atof( sValue.c_str() ) );

            getline ( m_pFileMag, sValue, ',' );
            pbVec->add_data( atof( sValue.c_str() ) );

            getline ( m_pFileMag, sValue );
            pbVec->add_data( atof( sValue.c_str() ) );
        }

        if( (m_bHaveAccel || m_bHaveGyro || m_bHaveMag) && m_IMUCallback ) {
            dataIMU.set_device_time(m_dNextTimePPS);
            dataIMU.set_system_time(m_dNextTime);
            m_IMUCallback( dataIMU );
        }

//        GPSData dataGPS;
//        dataGPS.data_present = 0;

//        if( pSelf->m_bHaveGPS ) {
//            dataGPS.data_present = IMU_GPS_LLH;
//            // TODO: this needs to be filled
//            // I didn't have a log to see the format
//        }

//        if( dataGPS.data_present && !pSelf->m_GPSCallback.empty() ) {
//            dataGPS.timestamp_system = pSelf->m_dNextTime;
//            dataIMU.timestamp_pps = pSelf->m_dNextTimePPS;
//            pSelf->m_GPSCallback( dataGPS );
//        }

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

    // Notify that this file has finished
    if (m_IMUFinishedCallback ){
      m_IMUFinishedCallback();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool CsvDriver::_GetNextTime(
        double& dNextTime,                  //< Output
        double& dNextTimePPS                //< Output
        )
{
    std::string sValue;

    getline ( m_pFileTime, sValue, ',' );
    dNextTime = atof( sValue.c_str() );
    getline ( m_pFileTime, sValue );
    dNextTimePPS = atof( sValue.c_str() );

    if( m_pFileTime.eof() ) {
        return false;
    }

    return true;
}

}
