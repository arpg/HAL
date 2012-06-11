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
void MicroStrainDriver::ImuCallback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
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
            while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
            {
                switch(field_header->descriptor)
                {
//                    // Scaled Accelerometer
//                    case MIP_AHRS_DATA_ACCEL_SCALED:
//                    {
//                        memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

//                        //For little-endian targets, byteswap the data field
//                        mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

//                        std::cout << "Accel\t" << curr_ahrs_accel.scaled_accel[0] << ", " << curr_ahrs_accel.scaled_accel[1] << ", " << curr_ahrs_accel.scaled_accel[2] << std::endl;
//                    }break;

//                        // Scaled Gyro
//                    case MIP_AHRS_DATA_GYRO_SCALED:
//                    {
//                        memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

//                        //For little-endian targets, byteswap the data field
//                        mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

//                        std::cout << "Gyro\t" << curr_ahrs_gyro.scaled_gyro[0] << ", " << curr_ahrs_gyro.scaled_gyro[1] << ", " << curr_ahrs_gyro.scaled_gyro[2] << std::endl;
//                    }break;

//                        // Scaled Magnetometer
//                    case MIP_AHRS_DATA_MAG_SCALED:
//                    {
//                        memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

//                        //For little-endian targets, byteswap the data field
//                        mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

//                        std::cout << "Magn\t" << curr_ahrs_mag.scaled_mag[0] << ", " << curr_ahrs_mag.scaled_mag[1] << ", " << curr_ahrs_mag.scaled_mag[2] << std::endl;
//                    }break;

                    case MIP_AHRS_DATA_QUATERNION:
                    {
                        mip_ahrs_quaternion curr_ahrs_quat;
                        memcpy(&curr_ahrs_quat, field_data, sizeof(mip_ahrs_quaternion));
                        mip_ahrs_quaternion_byteswap(&curr_ahrs_quat);

                        IMUData imuData = {
                            Eigen::Quaterniond(curr_ahrs_quat.q[0],curr_ahrs_quat.q[1],curr_ahrs_quat.q[2],curr_ahrs_quat.q[3])
                        };

                        if(!self->mCallback.empty()) {
                            self->mCallback(imuData);
                        }

//                        std::cout << "Quat\t" << imuData.rotation.coeffs() << std::endl;

                    }

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
    
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // Device communication parameters
    const u32 device_id = 0;
    const u32 baudrate = 115200;

    // Start up device
    if(mip_interface_init(device_id, baudrate, &mDeviceInterface,
                          DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
        return -1;
    }

    // Listen to commands only (no data output)
    while(mip_base_cmd_idle(&mDeviceInterface) != MIP_INTERFACE_OK) {
    }

    // Get device and print model name
    base_device_info_field device_info;
    char model_name[BASE_DEVICE_INFO_PARAM_LENGTH*2+1] = {0};
    while(mip_base_cmd_get_device_info(&mDeviceInterface, &device_info) != MIP_INTERFACE_OK) {
    }
    memcpy(model_name, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH*2);
    std::cout << "Found MicroStrain Device: " << model_name << std::endl;

    // Turn AHRS On
    u8 power_state = MIP_3DM_POWER_STATE_ON;
    while(mip_3dm_cmd_power_state(
              &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
              MIP_3DM_POWER_STATE_DEVICE_AHRS, &power_state) != MIP_INTERFACE_OK) {
    }

    // Specify Data we're interested in receiving (and level of decimation)
    u8  data_stream_format_descriptors[10] = {0};
    u16 data_stream_format_decimation[10]  = {0};
    data_stream_format_descriptors[0] = MIP_AHRS_DATA_QUATERNION;
//    data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
//    data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
//    data_stream_format_descriptors[2] = MIP_AHRS_DATA_MAG_SCALED;
    data_stream_format_decimation[0]  = 0x01;
//    data_stream_format_decimation[1]  = 0x0A;
//    data_stream_format_decimation[2]  = 0x0A;
    u8 data_stream_format_num_entries = 1;

    while(mip_3dm_cmd_ahrs_message_format(
              &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
              &data_stream_format_num_entries, data_stream_format_descriptors,
              data_stream_format_decimation) != MIP_INTERFACE_OK) {
    }

    // Setup callbacks for AHRS mode
    if(mip_interface_add_descriptor_set_callback(
           &mDeviceInterface, MIP_AHRS_DATA_SET, this /*NULL*/,
           &ImuCallback/*rpg_ahrs_packet_callback*/) != MIP_INTERFACE_OK)
        return -1;

    // Place in continuous ahrs mode
    u8 ahrs_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
              &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
              MIP_3DM_AHRS_DATASTREAM, &ahrs_enable) != MIP_INTERFACE_OK) {
    }

    // Resume data output (after idle mode)
    while(mip_base_cmd_resume(&mDeviceInterface) != MIP_INTERFACE_OK)
    {
    }

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

