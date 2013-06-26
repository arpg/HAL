#include "ProtoReaderIMUDriver.h"

#include <PbMsgs/Matrix.h>

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderIMUDriver::ProtoReaderIMUDriver(std::string filename)
    : m_callback(nullptr)
{
    m_reader = pb::Reader::Instance(filename);
    pb::Reader::ReadIMU = true;
}


/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderIMUDriver::_ThreadFunc()
{
    while( true ) {
        std::unique_ptr<pb::ImuMsg> readmsg = m_reader->ReadImuMsg();
        pb::ImuMsg test;
        if(readmsg) {
            // fill callback structure
            IMUData data;
            if( test.has_accel() ) {
                Eigen::VectorXd accel = pb::ReadVector(test.accel());
                data.accel = accel.cast<float>();
                data.data_present += IMU_AHRS_ACCEL;
            }
            if( test.has_gyro() ) {
                Eigen::VectorXd gyro = pb::ReadVector(test.gyro());
                data.gyro = gyro.cast<float>();
                data.data_present += IMU_AHRS_GYRO;
            }
            data.timestamp_pps = test.devicetime();
            m_callback( data );
        } else {
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderIMUDriver::~ProtoReaderIMUDriver()
{
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    m_callback = callback;
    m_callbackThread = std::thread( &ProtoReaderIMUDriver::_ThreadFunc, this );
}


