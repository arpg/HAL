#include <HAL/Utils/TicToc.h>

#include "NinjaIMUDriver.h"


using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NinjaIMUDriver::NinjaIMUDriver(const std::string& sCom)
    : m_Callback(nullptr), m_FtdiListener(FtdiListener::GetInstance()), m_Running(false)
{
    m_FtdiListener.Connect( sCom.c_str() );
}



/////////////////////////////////////////////////////////////////////////////////////////
NinjaIMUDriver::~NinjaIMUDriver()
{
    m_FtdiListener.Disconnect();
    m_Running = false;
    if( m_CallbackThread.joinable() ) {
        m_CallbackThread.join();
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void NinjaIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback Callback)
{
    m_Callback = Callback;
    m_Running = true;
    m_CallbackThread = std::thread( &NinjaIMUDriver::_ThreadFunc, this );
}


/////////////////////////////////////////////////////////////////////////////////////////
void NinjaIMUDriver::_ThreadFunc()
{
    pb::ImuMsg pbMsg;
    SensorPacket Pkt;
    while( m_Running ) {
        pbMsg.Clear();

        if( m_FtdiListener.ReadSensorPacket(Pkt) == 0 ) {
            std::cerr << "HAL: Error reading FTDI com port." << std::endl;
            continue;
        }

        pbMsg.set_id(1);
        pbMsg.set_device_time( hal::Tic() );

        pb::VectorMsg* pbVec = pbMsg.mutable_accel();
        pbVec->add_data(Pkt.Acc_x);
        pbVec->add_data(Pkt.Acc_y);
        pbVec->add_data(Pkt.Acc_z);

        pbVec = pbMsg.mutable_gyro();
        pbVec->add_data(Pkt.Gyro_x);
        pbVec->add_data(Pkt.Gyro_y);
        pbVec->add_data(Pkt.Gyro_z);

        pbVec = pbMsg.mutable_mag();
        pbVec->add_data(Pkt.Mag_x);
        pbVec->add_data(Pkt.Mag_y);
        pbVec->add_data(Pkt.Mag_z);

        m_Callback(pbMsg);
    }
}
