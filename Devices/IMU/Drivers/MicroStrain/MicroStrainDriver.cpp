#include "MicroStrainDriver.h"

#include <Eigen/Eigen>

#include <boost/thread.hpp>

#include "MIPSDK/mip_sdk.h"
#include "MIPSDK/mip_gx3_35.h"
#include "MIPSDK/byteswap_utilities.h"

using namespace boost;
using namespace std;
using namespace Eigen;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;

////AHRS
//mip_ahrs_scaled_gyro  curr_ahrs_gyro;
//mip_ahrs_scaled_accel curr_ahrs_accel;
//mip_ahrs_scaled_mag   curr_ahrs_mag;

#define _LINUX

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <time.h>
void Sleep(unsigned int time)
{
    struct timespec t,r;
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;
    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

double Tic()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}

///////////////////////////////////////////////////////////////////////////////
MicroStrainDriver::MicroStrainDriver()
    : mShouldRun(false)
{
}

///////////////////////////////////////////////////////////////////////////////
MicroStrainDriver::~MicroStrainDriver()
{
    mShouldRun = false;
    mDeviceThread.join();
}

///////////////////////////////////////////////////////////////////////////////
// C callback function for MIP library
///////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::ImuCallback(void *user_ptr, u8 *packet, u16 /*packet_size*/, u8 callback_type)
{
    MicroStrainDriver* self = static_cast<MicroStrainDriver*>(user_ptr);

    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch(callback_type)
    {
        case MIP_INTERFACE_CALLBACK_VALID_PACKET:
        {
            double dTimestamp = Tic();

            while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
            {
                switch(field_header->descriptor)
                {
                    // Scaled Accelerometer
                    case MIP_AHRS_DATA_ACCEL_SCALED:
                    {
                        mip_ahrs_scaled_accel curr_ahrs_accel;
                        memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

                        //For little-endian targets, byteswap the data field
                        mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

                        IMUData imuData;
                        imuData.type = IMU_AHRS_ACCEL;
                        imuData.timestamp = dTimestamp;
                        imuData.accel << curr_ahrs_accel.scaled_accel[0],
                                         curr_ahrs_accel.scaled_accel[1],
                                         curr_ahrs_accel.scaled_accel[2];

                        if(!self->mCallback.empty()){
                            self->mCallback(imuData);
                        }
                    }break;

                    // Scaled Gyro
                    case MIP_AHRS_DATA_GYRO_SCALED:
                    {
                        mip_ahrs_scaled_gyro curr_ahrs_gyro;
                        memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

                        //For little-endian targets, byteswap the data field
                        mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

                        IMUData imuData;
                        imuData.type = IMU_AHRS_GYRO;
                        imuData.timestamp = dTimestamp;
                        imuData.gyro << curr_ahrs_gyro.scaled_gyro[0],
                                        curr_ahrs_gyro.scaled_gyro[1],
                                        curr_ahrs_gyro.scaled_gyro[2];

                        if(!self->mCallback.empty()){
                            self->mCallback(imuData);
                        }
                    }break;

                     // Scaled Magnetometer
                    case MIP_AHRS_DATA_MAG_SCALED:
                    {
                        mip_ahrs_scaled_mag curr_ahrs_mag;

                        memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

                        //For little-endian targets, byteswap the data field
                        mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

                        IMUData imuData;
                        imuData.type = IMU_AHRS_MAG;
                        imuData.timestamp = dTimestamp;
                        imuData.mag << curr_ahrs_mag.scaled_mag[0],
                                        curr_ahrs_mag.scaled_mag[1],
                                        curr_ahrs_mag.scaled_mag[2];

                        if(!self->mCallback.empty()){
                            self->mCallback(imuData);
                        }

                    }break;

                    case MIP_AHRS_DATA_QUATERNION:
                    {
                        mip_ahrs_quaternion curr_ahrs_quat;
                        memcpy(&curr_ahrs_quat, field_data, sizeof(mip_ahrs_quaternion));
                        mip_ahrs_quaternion_byteswap(&curr_ahrs_quat);

                        IMUData imuData;
                        imuData.type = IMU_AHRS_QUATERNION;
                        imuData.timestamp = dTimestamp;
                        imuData.rotation = Eigen::Quaterniond(curr_ahrs_quat.q[0],curr_ahrs_quat.q[1],curr_ahrs_quat.q[2],curr_ahrs_quat.q[3]);

                        if(!self->mCallback.empty()) {
                            self->mCallback(imuData);
                        }


                    }break;

                    case MIP_AHRS_DATA_EULER_ANGLES:
                    {
                        mip_ahrs_euler_angles curr_ahrs_euler_angles;
                        memcpy(&curr_ahrs_euler_angles, field_data, sizeof(mip_ahrs_euler_angles));
                        mip_ahrs_euler_angles_byteswap(&curr_ahrs_euler_angles);

                        IMUData imuData;
                        imuData.type = IMU_AHRS_EULER;
                        imuData.timestamp = dTimestamp;
                        imuData.euler << curr_ahrs_euler_angles.roll,
                                         curr_ahrs_euler_angles.pitch,
                                         curr_ahrs_euler_angles.yaw;

                        if(!self->mCallback.empty()){
                            self->mCallback(imuData);
                        }

                    }break;

                    case MIP_GPS_DATA_LLH_POS:
                    {
                        mip_gps_llh_pos curr_gps_llh_pos;
                        memcpy(&curr_gps_llh_pos, field_data, sizeof(mip_gps_llh_pos));

                        mip_gps_llh_pos_byteswap(&curr_gps_llh_pos);
                        IMUData imuData;
                        imuData.type = IMU_GPS_LLH;
                        imuData.timestamp = dTimestamp;
                        imuData.llh << curr_gps_llh_pos.latitude,
                                       curr_gps_llh_pos.longitude,
                                       curr_gps_llh_pos.ellipsoid_height,
                                       curr_gps_llh_pos.msl_height,
                                       curr_gps_llh_pos.horizontal_accuracy,
                                       curr_gps_llh_pos.vertical_accuracy;

                        if(!self->mCallback.empty()){
                            self->mCallback(imuData);
                        }

                    }break;


                    default: break;
                }
            }
        }break;

            //Handle checksum error packets
        case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
        {
            std::cerr << "Checksome error!" << std::endl;
        }break;

            //Handle timeout packets
        case MIP_INTERFACE_CALLBACK_TIMEOUT:
        {
            std::cerr << "Timeout" << std::endl;
        }break;
        default: break;
    }
}

///////////////////////////////////////////////////////////////////////////////
bool MicroStrainDriver::Init()
{


    //*****************************************************************
    // Get driver properties
    //*****************************************************************
    m_bGetGPS  = m_pPropertyMap->GetProperty<bool>( "GetGPS", true );
    m_bGetAHRS = m_pPropertyMap->GetProperty<bool>( "GetAHRS", true );
    m_nHzGPS   = m_pPropertyMap->GetProperty<int>( "HzGPS",   1 );
    m_nHzAHRS  = m_pPropertyMap->GetProperty<int>( "HzAHRS", 100 );

    m_bGetEulerAHRS			= m_pPropertyMap->GetProperty<bool>( "GetEuler", true );
    m_bGetQuaternionAHRS	= m_pPropertyMap->GetProperty<bool>( "GetQuaternion", false );
    m_bGetAccelerometerAHRS = m_pPropertyMap->GetProperty<bool>( "GetAccelerometer", false );
    m_bGetGyroAHRS	        = m_pPropertyMap->GetProperty<bool>( "GetGyro", false );
    m_bGetMagnetometerAHRS  = m_pPropertyMap->GetProperty<bool>( "GetMagnetometer", false );
    //*****************************************************************

    assert(m_pPropertyMap);
    std::cout << "************************************************" << std::endl;
    std::cout << "Init MicroStrainDriver" << std::endl;
    std::cout << "************************************************" << std::endl;
    m_pPropertyMap->PrintPropertyMap();
    std::cout << "************************************************" << std::endl;

    // Device communication parameters
    const u32 device_id = 0;
    const u32 baudrate = 115200;


    // Start up device
    if(mip_interface_init(device_id, baudrate, &mDeviceInterface,
                          DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
        return -1;
    }

    // Listen to commands only (no data output)
    while(mip_base_cmd_idle(&mDeviceInterface) != MIP_INTERFACE_OK) { }

    // Get device and print model name
    base_device_info_field device_info;

    char model_name[BASE_DEVICE_INFO_PARAM_LENGTH*2+1] = {0};

    while(mip_base_cmd_get_device_info(&mDeviceInterface, &device_info) != MIP_INTERFACE_OK) { }

    memcpy(model_name, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH*2);

    if(m_bGetAHRS){
        if(!_ActivateAHRS())
            return false;
    }

    if(m_bGetGPS){
        if(!_ActivateGPS())
            return false;
    }

    // Resume data output (after idle mode)
    while(mip_base_cmd_resume(&mDeviceInterface) != MIP_INTERFACE_OK){ }

    mShouldRun = true;
    mDeviceThread = boost::thread(boost::bind(&MicroStrainDriver::_ThreadCaptureFunc,this));

    return true;
}

void MicroStrainDriver::RegisterDataCallback(IMUDriverDataCallback callback)
{
    mCallback = callback;
}

///////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::_ThreadCaptureFunc()
{
    while(mShouldRun){
        mip_interface_update(&mDeviceInterface);
        Sleep(1);
    }
    cout << "_ThreadCaptureFunc exiting" << endl;
}

///////////////////////////////////////////////////////////////////////////////
bool MicroStrainDriver::_ActivateAHRS()
{
    std::cout << "Activating AHRS..." << std::endl;

    // Turn AHRS On
    u8 power_state = MIP_3DM_POWER_STATE_ON;
    while(mip_3dm_cmd_power_state(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_POWER_STATE_DEVICE_AHRS, &power_state) != MIP_INTERFACE_OK) { }

    // Specify Data we're interested in receiving (and level of decimation)
    u8 ahrs_data_stream_format_num_entries;
    u8  ahrs_data_stream_format_descriptors[10] = {0};
    u16 ahrs_data_stream_format_decimation[10]  = {0};

    int idx		   = 0;
    int decimation = 1000/m_nHzAHRS;

    if(m_bGetEulerAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_EULER_ANGLES;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetQuaternionAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_QUATERNION;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetAccelerometerAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_ACCEL_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetGyroAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_GYRO_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetMagnetometerAHRS){
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_MAG_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }

    ahrs_data_stream_format_num_entries = idx;

    while(mip_3dm_cmd_ahrs_message_format(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        &ahrs_data_stream_format_num_entries, ahrs_data_stream_format_descriptors,
        ahrs_data_stream_format_decimation) != MIP_INTERFACE_OK) { }

    // Setup callbacks for AHRS mode
    if(mip_interface_add_descriptor_set_callback(
        &mDeviceInterface, MIP_AHRS_DATA_SET, this /*NULL*/,
        &ImuCallback/*rpg_ahrs_packet_callback*/) != MIP_INTERFACE_OK)
        return false;

    // Place in continuous ahrs mode
    u8 ahrs_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_AHRS_DATASTREAM, &ahrs_enable) != MIP_INTERFACE_OK) { }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
bool MicroStrainDriver::_ActivateGPS()
{
    std::cout << "Activating GPS..." << std::endl;
    u8 power_state = MIP_3DM_POWER_STATE_ON;

    // Turn GPS On
    while(mip_3dm_cmd_power_state(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_POWER_STATE_DEVICE_GPS, &power_state) != MIP_INTERFACE_OK) { }

    // Specify Data we're interested in receiving (and level of decimation)
    u8  gps_data_stream_format_num_entries	  = 1;
    u8  gps_data_stream_format_descriptors[1] = {0};
    u16 gps_data_stream_format_decimation[1]  = {0};

    gps_data_stream_format_descriptors[0]     = MIP_GPS_DATA_LLH_POS;
    gps_data_stream_format_decimation[0]      = 4/m_nHzGPS;

    while(mip_3dm_cmd_gps_message_format(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        &gps_data_stream_format_num_entries, gps_data_stream_format_descriptors,
        gps_data_stream_format_decimation) != MIP_INTERFACE_OK) { }

    // Setup callbacks for GPS mode
    if(mip_interface_add_descriptor_set_callback(
        &mDeviceInterface, MIP_GPS_DATA_SET, this, &ImuCallback) != MIP_INTERFACE_OK)
        return false;

    // Place in continuous GPS mode
    u8 gps_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
         &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
          MIP_3DM_GPS_DATASTREAM, &gps_enable) != MIP_INTERFACE_OK) { }

    return true;
}

