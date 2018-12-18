#include <iostream>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "MicroStrainDriver.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "MIPSDK/mip_sdk.h"
#include "MIPSDK/mip_gx3_35.h"
#include "MIPSDK/mip_gx4_45.h"
#include "MIPSDK/byteswap_utilities.h"
#pragma GCC diagnostic pop


using namespace hal;

IMUDriverDataCallback   MicroStrainDriver::mIMUCallback = nullptr;
PosysDriverDataCallback MicroStrainDriver::mPosysCallback = nullptr;

const int DEFAULT_PACKET_TIMEOUT_MS = 1000;

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
MicroStrainDriver::MicroStrainDriver(std::string portname,
                                     bool capture_accel,
                                     bool capture_gyro,
                                     bool capture_mag,
                                     bool capture_gps,
                                     int gps_hz,
                                     int imu_hz)
    : mShouldRun(false),
      m_bGetGPS(capture_gps),
      m_bGetAccelerometerAHRS(capture_accel),
      m_bGetGyroAHRS(capture_gyro),
      m_bGetMagnetometerAHRS(capture_mag),
      m_nHzGPS(gps_hz),
      m_nHzAHRS(imu_hz),
      m_sPortName(portname)
{
    m_bGetEulerAHRS			= false;
    m_bGetQuaternionAHRS	= false;
    m_bGetAHRS = capture_accel || capture_gyro || capture_mag;
    m_bGetTimeStampPpsAHRS  = true;
}

///////////////////////////////////////////////////////////////////////////////
MicroStrainDriver::~MicroStrainDriver()
{
    mShouldRun = false;
    mDeviceThread.join();

    if(mDeviceInterface.port_handle)
    {
        if(m_bGetAHRS) {
            // Take out of continuous ahrs mode
            u8 ahrs_enable = 0;
            while(mip_3dm_cmd_continuous_data_stream(
                &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
                MIP_3DM_AHRS_DATASTREAM, &ahrs_enable) != MIP_INTERFACE_OK) { }
        }

        if(m_bGetGPS) {
            // Take out of continuous GPS mode
            u8 gps_enable = 0;
            while(mip_3dm_cmd_continuous_data_stream(
                 &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
                  MIP_3DM_GPS_DATASTREAM, &gps_enable) != MIP_INTERFACE_OK) { }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// C callback function for MIP library
///////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::CallbackFunc(void* /*user_ptr*/, u8 *packet, u16 /*packet_size*/, u8 callback_type)
{
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch(callback_type)
    {
        case MIP_INTERFACE_CALLBACK_VALID_PACKET:
        {
            hal::ImuMsg pbImuMsg;
            hal::PoseMsg pbPoseMsg;

            pbImuMsg.set_system_time(hal::Tic());
            pbPoseMsg.set_system_time(hal::Tic());

            // Read each field in packet.
            // For little-endian targets, byteswap the data field
            while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
            {
                switch(field_header->descriptor)
                {
                    // get pps timestamp, not available on gx5-15
                    case MIP_AHRS_DATA_TIME_STAMP_PPS:
                    {
                        mip_ahrs_1pps_timestamp curr_ahrs_pps_timestamp;
                        memcpy(&curr_ahrs_pps_timestamp, field_data, sizeof(mip_ahrs_1pps_timestamp));
                        mip_ahrs_1pps_timestamp_byteswap(&curr_ahrs_pps_timestamp);
                        pbImuMsg.set_device_time(curr_ahrs_pps_timestamp.seconds + 1E-9 * curr_ahrs_pps_timestamp.nanoseconds);

                        // TODO It is probably better to use a differnent time specific for the GPS??
                        pbPoseMsg.set_device_time(curr_ahrs_pps_timestamp.seconds + 1E-9 * curr_ahrs_pps_timestamp.nanoseconds);
                    }break;

                    // get gps time, only time available with gx5-15
                    case MIP_AHRS_DATA_TIME_STAMP_GPS:
                    {
                        mip_ahrs_gps_timestamp curr_ahrs_gps_timestamp;
                        memcpy(&curr_ahrs_gps_timestamp, field_data, sizeof(mip_ahrs_gps_timestamp));
                        mip_ahrs_gps_timestamp_byteswap(&curr_ahrs_gps_timestamp);
                        pbImuMsg.set_device_time(curr_ahrs_gps_timestamp.tow);
                        pbPoseMsg.set_device_time(curr_ahrs_gps_timestamp.tow);
                    }break;

                    // Scaled Accelerometer
                    case MIP_AHRS_DATA_ACCEL_SCALED:
                    {
                        mip_ahrs_scaled_accel curr_ahrs_accel;
                        memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));
                        mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

                        hal::VectorMsg* pbVec = pbImuMsg.mutable_accel();
                        pbVec->add_data(curr_ahrs_accel.scaled_accel[0] *
                            9.80665);
                        pbVec->add_data(curr_ahrs_accel.scaled_accel[1] *
                            9.80665);
                        pbVec->add_data(curr_ahrs_accel.scaled_accel[2] *
                            9.80665);
                    }break;

                    // Scaled Gyro
                    case MIP_AHRS_DATA_GYRO_SCALED:
                    {
                        mip_ahrs_scaled_gyro curr_ahrs_gyro;
                        memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));
                        mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

                        hal::VectorMsg* pbVec = pbImuMsg.mutable_gyro();
                        pbVec->add_data(curr_ahrs_gyro.scaled_gyro[0]);
                        pbVec->add_data(curr_ahrs_gyro.scaled_gyro[1]);
                        pbVec->add_data(curr_ahrs_gyro.scaled_gyro[2]);
                    }break;

                     // Scaled Magnetometer
                    case MIP_AHRS_DATA_MAG_SCALED:
                    {
                        mip_ahrs_scaled_mag curr_ahrs_mag;
                        memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));
                        mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

                        hal::VectorMsg* pbVec = pbImuMsg.mutable_mag();
                        pbVec->add_data(curr_ahrs_mag.scaled_mag[0]);
                        pbVec->add_data(curr_ahrs_mag.scaled_mag[1]);
                        pbVec->add_data(curr_ahrs_mag.scaled_mag[2]);
                    }break;

                    case MIP_AHRS_DATA_QUATERNION:
                    {
                        mip_ahrs_quaternion curr_ahrs_quat;
                        memcpy(&curr_ahrs_quat, field_data, sizeof(mip_ahrs_quaternion));
                        mip_ahrs_quaternion_byteswap(&curr_ahrs_quat);

//                        imuData.data_present |= IMU_AHRS_QUATERNION;
//                        imuData.rotation = Eigen::Quaterniond(curr_ahrs_quat.q[0],curr_ahrs_quat.q[1],curr_ahrs_quat.q[2],curr_ahrs_quat.q[3]);
                    }break;

                    case MIP_AHRS_DATA_EULER_ANGLES:
                    {
                        mip_ahrs_euler_angles curr_ahrs_euler_angles;
                        memcpy(&curr_ahrs_euler_angles, field_data, sizeof(mip_ahrs_euler_angles));
                        mip_ahrs_euler_angles_byteswap(&curr_ahrs_euler_angles);

//                        imuData.data_present |= IMU_AHRS_EULER;
//                        imuData.euler << curr_ahrs_euler_angles.roll,
//                                         curr_ahrs_euler_angles.pitch,
//                                         curr_ahrs_euler_angles.yaw;
                    }break;

                    case MIP_GPS_DATA_LLH_POS:
                    {
                        mip_gps_llh_pos curr_gps_llh_pos;
                        memcpy(&curr_gps_llh_pos, field_data, sizeof(mip_gps_llh_pos));
                        mip_gps_llh_pos_byteswap(&curr_gps_llh_pos);

                        pbPoseMsg.set_type( hal::PoseMsg_Type_LatLongAlt );

                        hal::VectorMsg* pbVec = pbPoseMsg.mutable_pose();

                        pbVec->add_data(curr_gps_llh_pos.latitude);
                        pbVec->add_data(curr_gps_llh_pos.longitude);
                        pbVec->add_data(curr_gps_llh_pos.ellipsoid_height);

                        /*
                        gpsData.data_present |= IMU_GPS_LLH;
                        gpsData.llh << curr_gps_llh_pos.latitude,
                                       curr_gps_llh_pos.longitude,
                                       curr_gps_llh_pos.ellipsoid_height,
                                       curr_gps_llh_pos.msl_height,
                                       curr_gps_llh_pos.horizontal_accuracy,
                                       curr_gps_llh_pos.vertical_accuracy;
                        */
                    }break;

                    default: break;
                }
            }

            if(mIMUCallback){
              if(pbImuMsg.has_accel() || pbImuMsg.has_gyro() || pbImuMsg.has_mag() ) {
                mIMUCallback(pbImuMsg);
              }
            }
            if(mPosysCallback){
              if(pbPoseMsg.has_pose()) {
                mPosysCallback(pbPoseMsg);
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
bool MicroStrainDriver::_Init()
{

    // Device communication parameters
    const u32 device_id = 0;
    const u32 baudrate = 115200;


    // Start up device
    if(mip_interface_init(device_id, m_sPortName, baudrate, &mDeviceInterface,
                          DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
        return false;
    }

    // Get device info and model name
    base_device_info_field device_info;
    char model_name[BASE_DEVICE_INFO_PARAM_LENGTH*2+1] = {0};
    while(mip_base_cmd_get_device_info(&mDeviceInterface, &device_info) != MIP_INTERFACE_OK) { }
    memcpy(model_name, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH*2);

    // trim whitespace from name string
    std::string name_string(model_name);
    size_t str_start = name_string.find_first_not_of(' ');
    size_t str_end = name_string.find_last_not_of(' ');
    name_string = name_string.substr(str_start, (str_end - str_start + 1));

    std::cout << "model name: " << name_string << std::endl;

    // TO DO: set time and magnetometer fields based on model name
    //        just using values for gx5-15 for now
    if (name_string.find("GX5") != std::string::npos)
    {
        std::cout << "PPS timestamp not supported on GX5, switching to GPS timestamp" << std::endl;
        m_bGetTimeStampGpsCorrelationAHRS = true;
        m_bGetTimeStampPpsAHRS = false;
        if (m_bGetMagnetometerAHRS)
        {
            std::cout << "magnetometer not supported on GX5, setting magnetometer to false";
            m_bGetMagnetometerAHRS = false;
        }
    }

    if (mip_interface_add_descriptor_set_callback(&mDeviceInterface, MIP_AHRS_DATA_SET, this, &CallbackFunc)
        != MIP_INTERFACE_OK)
        return false;

    u8 com_mode = MIP_SDK_GX4_45_IMU_STANDARD_MODE;
    while (mip_system_com_mode(&mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) 
        != MIP_INTERFACE_OK);

    while (mip_system_com_mode(&mDeviceInterface, MIP_FUNCTION_SELECTOR_READ, &com_mode)
        != MIP_INTERFACE_OK);

    if (com_mode != MIP_SDK_GX4_45_IMU_STANDARD_MODE)
        return false;

    while (mip_base_cmd_idle(&mDeviceInterface) != MIP_INTERFACE_OK);
    //std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
    mDeviceThread = std::thread(std::bind(&MicroStrainDriver::_ThreadCaptureFunc,this));

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    mIMUCallback = callback;
    if(_Init() == false) {
      throw DeviceException("Error initializing IMU.");
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::RegisterPosysDataCallback(PosysDriverDataCallback callback)
{
  // TODO(jmf) This was done in a rush (typical). If the user only wants GPS
  // and the IMU is not initialized at some point, no GPS data will be received!
  // Make this a singleton which can be initialized by either IMU or GPS driver.
  mPosysCallback = callback;
}


///////////////////////////////////////////////////////////////////////////////
void MicroStrainDriver::_ThreadCaptureFunc()
{
    while(mShouldRun){
        mip_interface_update(&mDeviceInterface);
    }
    mShouldRun = false;
}

///////////////////////////////////////////////////////////////////////////////
bool MicroStrainDriver::_ActivateAHRS()
{
    std::cout << "Activating AHRS..." << std::endl;

    u16 base_rate = 0;
    while (mip_3dm_cmd_get_ahrs_base_rate(&mDeviceInterface, &base_rate) != MIP_INTERFACE_OK);
    u8 decimation = (u8)((float)base_rate/(float)m_nHzAHRS);

    u8 ahrs_data_stream_format_num_entries;
    u8  ahrs_data_stream_format_descriptors[10] = {0};
    u16 ahrs_data_stream_format_decimation[10]  = {0};

    int idx = 0;
    if (m_bGetTimeStampPpsAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_TIME_STAMP_PPS;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if (m_bGetTimeStampGpsCorrelationAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_TIME_STAMP_GPS;
        ahrs_data_stream_format_decimation[idx] = decimation;
        idx++;
    }
    if(m_bGetEulerAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_EULER_ANGLES;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetQuaternionAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_QUATERNION;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetAccelerometerAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_ACCEL_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetGyroAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_GYRO_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetMagnetometerAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_MAG_SCALED;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }

    ahrs_data_stream_format_num_entries = idx;

    while (mip_3dm_cmd_ahrs_message_format(&mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE, &ahrs_data_stream_format_num_entries, ahrs_data_stream_format_descriptors, ahrs_data_stream_format_decimation) != MIP_INTERFACE_OK);

    while (mip_3dm_cmd_poll_ahrs(&mDeviceInterface, MIP_3DM_POLLING_ENABLE_ACK_NACK, ahrs_data_stream_format_num_entries, ahrs_data_stream_format_descriptors) != MIP_INTERFACE_OK);

    u8 enable = 0x01;
    while (mip_3dm_cmd_continuous_data_stream(&mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK);

    return true; 

    /* Compatible with gx3-35
    // Turn AHRS On
    u8 power_state = MIP_3DM_POWER_STATE_ON;
    std::cout << "setting power state" << std::endl;
    
    while(mip_3dm_cmd_power_state(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_POWER_STATE_DEVICE_AHRS, &power_state) != MIP_INTERFACE_OK) { }
    
    std::cout << "power state set" << std::endl;

    // Specify Data we're interested in receiving (and level of decimation)
    u8 ahrs_data_stream_format_num_entries;
    u8  ahrs_data_stream_format_descriptors[10] = {0};
    u16 ahrs_data_stream_format_decimation[10]  = {0};

    int idx		   = 0;
    int decimation = 1000/m_nHzAHRS;

    if(m_bGetTimeStampPpsAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_TIME_STAMP_PPS;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
    if(m_bGetTimeStampGpsCorrelationAHRS)
    {
        ahrs_data_stream_format_descriptors[idx] = MIP_AHRS_DATA_TIME_STAMP_GPS;
        ahrs_data_stream_format_decimation[idx]  = decimation;
        idx++;
    }
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

    std::cout << "setting message format with " << idx << " descriptors" << std::endl;
    
    while(mip_3dm_cmd_ahrs_message_format(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        &ahrs_data_stream_format_num_entries, ahrs_data_stream_format_descriptors,
        ahrs_data_stream_format_decimation) != MIP_INTERFACE_OK) { }
    
    std::cout << "setting callback" << std::endl;

    // Setup callbacks for AHRS mode
    if(mip_interface_add_descriptor_set_callback(
        &mDeviceInterface, MIP_AHRS_DATA_SET, this,
        &CallbackFunc) != MIP_INTERFACE_OK)
        return false;

    std::cout << "setting continuous mode" << std::endl;

    // Place in continuous ahrs mode
    u8 ahrs_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
        &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
        MIP_3DM_AHRS_DATASTREAM, &ahrs_enable) != MIP_INTERFACE_OK) { }

    std::cout << "done activating AHRS" << std::endl;

    return true;

    */
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
        &mDeviceInterface, MIP_GPS_DATA_SET, this, &CallbackFunc) != MIP_INTERFACE_OK)
        return false;

    // Place in continuous GPS mode
    u8 gps_enable = 1;
    while(mip_3dm_cmd_continuous_data_stream(
         &mDeviceInterface, MIP_FUNCTION_SELECTOR_WRITE,
          MIP_3DM_GPS_DATASTREAM, &gps_enable) != MIP_INTERFACE_OK) { }

    return true;
}
