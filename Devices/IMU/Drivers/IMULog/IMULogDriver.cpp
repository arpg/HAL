#include "IMULogDriver.h"

#include <RPG/Devices/VirtualDevice.h>


/////////////////////////////////////////////////////////////////////////////////////////
IMULogDriver::IMULogDriver()
{
    m_bShouldRun = false;
    m_bHaveAccel = false;
    m_bHaveGyro = false;
    m_bHaveMag = false;
    m_bHaveGPS = false;

    m_dNextTime = 0;
    m_dNextTimePPS = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
IMULogDriver::~IMULogDriver()
{
    // close capture thread
    m_bShouldRun = false;

    // this is fugly, but we need to wake up the sleeping capture thread somehow
    VirtualDevice::CONDVAR.notify_all();

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
bool IMULogDriver::Init()
{
    std::string sDataSourceDir  = m_pPropertyMap->GetProperty( "DataSourceDir", "");

    std::string sFileAccel      = m_pPropertyMap->GetProperty( "Accel", "/imu/accel.txt");
    sFileAccel = sDataSourceDir + sFileAccel;

    std::string sFileGyro       = m_pPropertyMap->GetProperty( "Gyro", "/imu/gyro.txt");
    sFileGyro = sDataSourceDir + sFileGyro;

    std::string sFileMag        = m_pPropertyMap->GetProperty( "Mag", "/imu/mag.txt");
    sFileMag = sDataSourceDir + sFileMag;

    std::string sFileGPS        = m_pPropertyMap->GetProperty( "GPS", "/imu/gps.txt");
    sFileGPS = sDataSourceDir + sFileGPS;

    std::string sFileTimestamp  = m_pPropertyMap->GetProperty( "Timestamp", "/imu/timestamp.txt");
    sFileTimestamp = sDataSourceDir + sFileTimestamp;

    // open IMU log files
    if( sFileTimestamp.empty() ) {
        std::cerr << "IMULog: File with timestamps is required." << std::endl;
        return false;
    } else {
        m_pFileTime.open( sFileTimestamp.c_str() );
        if( m_pFileTime.is_open() == false ) {
            std::cerr << "IMULog: File with timestamps could not be opened." << std::endl;
            return false;
        }
    }

    if( sFileAccel.empty() == false ) {
        m_pFileAccel.open( sFileAccel.c_str() );
        if( m_pFileTime.is_open() == false ) {
            std::cerr << "IMULog: File with accel info could not be opened." << std::endl;
        } else {
            m_bHaveAccel = true;
        }
    }

    if( sFileGyro.empty() == false ) {
        m_pFileGyro.open( sFileGyro.c_str() );
        if( m_pFileTime.is_open() == false ) {
            std::cerr << "IMULog: File with gyro info could not be opened." << std::endl;
        } else {
            m_bHaveGyro = true;
        }
    }

    if( sFileMag.empty() == false ) {
        m_pFileMag.open( sFileMag.c_str() );
        if( m_pFileTime.is_open() == false ) {
            std::cerr << "IMULog: File with mag info could not be opened." << std::endl;
        } else {
            m_bHaveMag = true;
        }
    }

    if( sFileGPS.empty() == false ) {
        m_pFileGPS.open( sFileGPS.c_str() );
        if( m_pFileTime.is_open() == false ) {
            std::cerr << "IMULog: File with GPS info could not be opened." << std::endl;
        } else {
            m_bHaveGPS = true;
        }
    }

    // read one timestamp.. return false if error occurs
    if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
        return false;
    }

    // push timestamp to VD queue
    VirtualDevice::PushTime( m_dNextTime );

    // start capture thread
    m_bShouldRun = true;
    m_DeviceThread = boost::thread( &IMULogDriver::_ThreadCaptureFunc, this );

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void IMULogDriver::RegisterDataCallback( IMUDriverDataCallback callback )
{
    m_IMUCallback = callback;
}

/////////////////////////////////////////////////////////////////////////////////////////
void IMULogDriver::RegisterDataCallback( GPSDriverDataCallback callback )
{
    m_GPSCallback = callback;
}

/////////////////////////////////////////////////////////////////////////////////////////
void IMULogDriver::_ThreadCaptureFunc( IMULogDriver* pSelf )
{
    while( pSelf->m_bShouldRun ) {

        boost::mutex::scoped_lock vd_lock(VirtualDevice::MUTEX);

        // check if timestamp is the top of the queue
        while( VirtualDevice::NextTime() < pSelf->m_dNextTime ) {
            VirtualDevice::CONDVAR.wait( vd_lock );

            // check if this variable has changed during our sleep
            if( pSelf->m_bShouldRun == false ) {
                // oops, someone wants use dead
                goto DIE;
            }
        }
        // sweet, we are good to go!
        vd_lock.unlock();


        //---------------------------------------------------------

        // get data and pack
        // we already have the timestamp so no need to read it!
        std::string     sValue;
        float           fValue;

        IMUData dataIMU;
        dataIMU.data_present = 0;

        if( pSelf->m_bHaveAccel ) {
            dataIMU.data_present = dataIMU.data_present | IMU_AHRS_ACCEL;
            getline ( pSelf->m_pFileAccel, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.accel(0) = fValue;
            getline ( pSelf->m_pFileAccel, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.accel(1) = fValue;
            getline ( pSelf->m_pFileAccel, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.accel(2) = fValue;
            getline ( pSelf->m_pFileAccel, sValue );
        }

        // TODO: add "euler"

        if( pSelf->m_bHaveGyro ) {
            dataIMU.data_present = dataIMU.data_present | IMU_AHRS_GYRO;
            getline ( pSelf->m_pFileGyro, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.gyro(0) = fValue;
            getline ( pSelf->m_pFileGyro, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.gyro(1) = fValue;
            getline ( pSelf->m_pFileGyro, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.gyro(2) = fValue;
            getline ( pSelf->m_pFileGyro, sValue );
        }

        if( pSelf->m_bHaveMag ) {
            dataIMU.data_present = dataIMU.data_present | IMU_AHRS_MAG;
            getline ( pSelf->m_pFileMag, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.mag(0) = fValue;
            getline ( pSelf->m_pFileMag, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.mag(1) = fValue;
            getline ( pSelf->m_pFileMag, sValue, ',' );
            fValue = atof( sValue.c_str() );
            dataIMU.mag(2) = fValue;
            getline ( pSelf->m_pFileMag, sValue );
        }

        if( dataIMU.data_present && !pSelf->m_IMUCallback.empty() ) {
            dataIMU.timestamp_system = pSelf->m_dNextTime;
            dataIMU.timestamp_pps = pSelf->m_dNextTimePPS;
            pSelf->m_IMUCallback( dataIMU );
        }




        GPSData dataGPS;
        dataGPS.data_present = 0;

        if( pSelf->m_bHaveGPS ) {
            dataGPS.data_present = IMU_GPS_LLH;
            // TODO: this needs to be filled
            // I didn't have a log to see the format
        }

        if( dataGPS.data_present && !pSelf->m_GPSCallback.empty() ) {
            dataGPS.timestamp_system = pSelf->m_dNextTime;
            dataIMU.timestamp_pps = pSelf->m_dNextTimePPS;
            pSelf->m_GPSCallback( dataGPS );
        }

        //---------------------------------------------------------


        // break if EOF
        if( pSelf->_GetNextTime( pSelf->m_dNextTime, pSelf->m_dNextTimePPS ) == false ) {
            goto DIE;
        }

        // pop front and push next timestamp to queue
        VirtualDevice::PopAndPushTime( pSelf->m_dNextTime );
    }

    DIE:
    VirtualDevice::PopTime();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool IMULogDriver::_GetNextTime(
        double& dNextTime,                  //< Output
        double& dNextTimePPS                //< Output
        )
{
    std::string sValue;
    if( m_pFileTime.eof() ) {
        return false;
    }
    getline ( m_pFileTime, sValue, ',' );
    dNextTime = atof( sValue.c_str() );
    getline ( m_pFileTime, sValue );
    dNextTimePPS = atof( sValue.c_str() );
    return true;
}
