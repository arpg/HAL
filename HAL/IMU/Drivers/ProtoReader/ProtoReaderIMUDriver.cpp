#include "ProtoReaderIMUDriver.h"

#include <PbMsgs/Matrix.h>

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderIMUDriver::ProtoReaderIMUDriver(std::string filename)
    : m_reader(pb::Reader::Instance(filename,pb::Msg_Type_IMU)), m_running(false), m_callback(nullptr)
{
}


/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderIMUDriver::_ThreadFunc()
{
    while( m_running ) {
        std::unique_ptr<pb::ImuMsg> readmsg = m_reader.ReadImuMsg();
        if(readmsg) {
            // fill callback structure
            IMUData data;
            if( readmsg->has_accel() ) {
                Eigen::VectorXd accel = pb::ReadVector(readmsg->accel());
                data.accel = accel.cast<float>();
                data.data_present += IMU_AHRS_ACCEL;
            }
            if( readmsg->has_gyro() ) {
                Eigen::VectorXd gyro = pb::ReadVector(readmsg->gyro());
                data.gyro = gyro.cast<float>();
                data.data_present += IMU_AHRS_GYRO;
            }
            data.timestamp_pps = readmsg->devicetime();
            m_callback( data );
        } else {
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
ProtoReaderIMUDriver::~ProtoReaderIMUDriver()
{
    m_running = false;
    m_reader.StopBuffering();
    m_callbackThread.join();
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProtoReaderIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &ProtoReaderIMUDriver::_ThreadFunc, this );
}


