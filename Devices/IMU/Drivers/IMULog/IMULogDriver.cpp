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
}

/////////////////////////////////////////////////////////////////////////////////////////
IMULogDriver::~IMULogDriver()
{
    // close capture thread
    m_bShouldRun = false;
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
        std::cout << "Timestamp file is: " << sFileTimestamp << std::endl;
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

    // read once and push timestamp to VD queue
    m_dNextTime = _GetNextTime();
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

        /*
        // get data and pack
        // we already have the timestamp so no need to read it!
        std::stringstream sValue;

        IMUData& dataIMU = pSelf->m_NextData;
        dataIMU.data_present = 0;

        if( pSelf->m_bHaveAccel ) {
            dataIMU.data_present = dataIMU.data_present | IMU_AHRS_ACCEL;
        }

        if( pSelf->m_bHaveGyro ) {
            dataIMU.data_present = dataIMU.data_present | IMU_AHRS_GYRO;
        }

        if( pSelf->m_bHaveMag ) {
            dataIMU.data_present = dataIMU.data_present | IMU_AHRS_MAG;
        }

        if( dataIMU.data_present && !pSelf->m_IMUCallback.empty() ) {
            dataIMU.timestamp_system = 00000;
            pSelf->m_IMUCallback( dataIMU );
        }




        GPSData dataGPS;
        dataGPS.data_present = 0;
        dataGPS.timestamp_system = 0;

        if( pSelf->m_bHaveGPS ) {
            dataGPS.data_present = IMU_GPS_LLH;
        }

        if( dataGPS.data_present && !pSelf->m_GPSCallback.empty() ) {
            dataGPS.timestamp_system = 00000;
            pSelf->m_GPSCallback( dataGPS );
        }
        */


        // push next timestamp to queue now that we popped from the buffer
        pSelf->m_dNextTime = pSelf->_GetNextTime();

        // break if EOF
        if( pSelf->m_dNextTime == -1 ) {
            goto DIE;
        }

        VirtualDevice::PopAndPushTime( pSelf->m_dNextTime );
    }

    DIE:
    VirtualDevice::PopTime();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double IMULogDriver::_GetNextTime()
{
    double time;
    std::string sValue;
    if( m_pFileTime.eof() ) {
        return -1;
    }
    getline ( m_pFileTime, sValue, ',' );
    time = atof( sValue.c_str() );
    getline ( m_pFileTime, sValue );
    return time;
}
