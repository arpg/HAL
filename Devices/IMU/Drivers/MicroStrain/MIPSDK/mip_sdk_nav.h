/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_nav.h 
//! @author  Nathan Miller
//! @version 1.0
//
//! @description NAV MIP Descriptor Set Definitions.
//
// External dependencies:
//
//  
// 
//! @copyright 2011 Microstrain. 
//
//!@section CHANGES
//! 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE 
//! FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY 
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY 
//! CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH 
//! THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


#ifndef _MIP_SDK_NAV_H
#define _MIP_SDK_NAV_H


////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip.h"
#include "mip_sdk_interface.h"

////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
////////////////////////////////////////////////////////////////////////////////
//! @def 


////////////////////////////////////////////////////////////////////////////////
//
// Descriptor Set designators - used in the Desc Set field of the MIP header
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_NAV_COMMAND_SET								0x0D
#define MIP_NAV_DATA_SET								0x82


////////////////////////////////////////////////////////////////////////////////
// NAV COMMAND DESCRIPTORS (command desc are < 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_NAV_CMD_RESET_FILTER                   0x01
#define MIP_NAV_CMD_SET_INITIAL_ATTITUDE           0x02
#define MIP_NAV_CMD_SET_INITIAL_HEADING            0x03
#define MIP_NAV_CMD_SET_INITIAL_HEADING_FROM_AHRS  0x04

#define MIP_NAV_CMD_VEHICLE_DYNAMICS_MODE          0x10
#define MIP_NAV_CMD_SENSOR2VEHICLE_TRANSFORMATION  0x11
#define MIP_NAV_CMD_SENSOR2VEHICLE_OFFSET          0x12
#define MIP_NAV_CMD_ANTENNA_OFFSET                 0x13
#define MIP_NAV_CMD_BIAS_ESTIMATION_CONTROL        0x14

#define MIP_NAV_CMD_GPS_SOURCE_CONTROL             0x15
#define MIP_NAV_CMD_EXTERNAL_GPS_UPDATE            0x16
#define MIP_NAV_CMD_EXTERNAL_HEADING_UPDATE        0x17
#define MIP_NAV_CMD_HEADING_UPDATE_CONTROL         0x18
#define MIP_NAV_CMD_AUTOINIT_CONTROL               0x19
#define MIP_NAV_CMD_ACCEL_NOISE                    0x1A
#define MIP_NAV_CMD_GYRO_NOISE                     0x1B
#define MIP_NAV_CMD_ACCEL_BIAS_MODEL               0x1C
#define MIP_NAV_CMD_GYRO_BIAS_MODEL                0x1D


////////////////////////////////////////////////////////////////////////////////
// NAV REPLY DESCRIPTORS (reply desc are >= 0x80)
////////////////////////////////////////////////////////////////////////////////

#define MIP_NAV_REPLY_VEHICLE_DYNAMICS_MODE          0x80
#define MIP_NAV_REPLY_SENSOR2VEHICLE_TRANSFORMATION  0x81
#define MIP_NAV_REPLY_SENSOR2VEHICLE_OFFSET          0x82
#define MIP_NAV_REPLY_ANTENNA_OFFSET                 0x83
#define MIP_NAV_REPLY_BIAS_ESTIMATION_CONTROL        0x84
#define MIP_NAV_REPLY_GPS_SOURCE_CONTROL             0x86
#define MIP_NAV_REPLY_HEADING_UPDATE_CONTROL         0x87
#define MIP_NAV_REPLY_AUTOINIT_CONTROL               0x88
#define MIP_NAV_REPLY_ACCEL_NOISE                    0x89
#define MIP_NAV_REPLY_GYRO_NOISE                     0x8A
#define MIP_NAV_REPLY_ACCEL_BIAS_MODEL               0x8B
#define MIP_NAV_REPLY_GYRO_BIAS_MODEL                0x8C


////////////////////////////////////////////////////////////////////////////////
//
// NAV DATA DESCRIPTORS
//
////////////////////////////////////////////////////////////////////////////////

#define MIP_NAV_DATA_LLH_POS          	         0x01	 
#define MIP_NAV_DATA_NED_VEL		      	     0x02	
#define MIP_NAV_DATA_ATT_QUATERNION      	     0x03	 
#define MIP_NAV_DATA_ATT_MATRIX   		  	     0x04	 
#define MIP_NAV_DATA_ATT_EULER_ANGLES  			 0x05	 
#define MIP_NAV_DATA_GYRO_BIAS    		  	     0x06	 
#define MIP_NAV_DATA_ACCEL_BIAS  		  	     0x07	 
#define MIP_NAV_DATA_POS_UNCERTAINTY	  	     0x08	 
#define MIP_NAV_DATA_VEL_UNCERTAINTY 	         0x09	
#define MIP_NAV_DATA_ATT_UNCERTAINTY_EULER       0x0A	 
#define MIP_NAV_DATA_GYRO_BIAS_UNCERTAINTY       0x0B	
#define MIP_NAV_DATA_ACCEL_BIAS_UNCERTAINTY      0x0C	  
#define MIP_NAV_DATA_ACCELERATION                0x0D	
#define MIP_NAV_DATA_ANGULAR_RATE                0x0E	  
#define MIP_NAV_DATA_WGS84_GRAVITY               0x0F	  
#define MIP_NAV_DATA_FILTER_STATUS               0x10	
#define MIP_NAV_DATA_FILTER_TIMESTAMP            0x11	 
#define MIP_NAV_DATA_ATT_UNCERTAINTY_QUATERNION  0x12	 
#define MIP_NAV_DATA_GRAVITY_VECTOR              0x13	
#define MIP_NAV_DATA_HEADING_UPDATE_STATE        0x14
#define MIP_NAV_DATA_MAGNETIC_MODEL              0x15	  


//NAV Data Descriptor Validation Macro
#define IS_NAV_DATA_DESCRIPTOR(DESC) (((DESC) == MIP_NAV_DATA_LLH_POS)                     || \
                                      ((DESC) == MIP_NAV_DATA_NED_VEL)                     || \
                                      ((DESC) == MIP_NAV_DATA_ATT_QUATERNION)              || \
                                      ((DESC) == MIP_NAV_DATA_ATT_MATRIX)                  || \
                                      ((DESC) == MIP_NAV_DATA_ATT_EULER_ANGLES)            || \
                                      ((DESC) == MIP_NAV_DATA_GYRO_BIAS)                   || \
                                      ((DESC) == MIP_NAV_DATA_ACCEL_BIAS)                  || \
                                      ((DESC) == MIP_NAV_DATA_POS_UNCERTAINTY)             || \
                                      ((DESC) == MIP_NAV_DATA_VEL_UNCERTAINTY)             || \
                                      ((DESC) == MIP_NAV_DATA_ATT_UNCERTAINTY_EULER)       || \
                                      ((DESC) == MIP_NAV_DATA_GYRO_BIAS_UNCERTAINTY)       || \
                                      ((DESC) == MIP_NAV_DATA_ACCEL_BIAS_UNCERTAINTY)      || \
                                      ((DESC) == MIP_NAV_DATA_ACCELERATION)                || \
                                      ((DESC) == MIP_NAV_DATA_ANGULAR_RATE)                || \
                                      ((DESC) == MIP_NAV_DATA_WGS84_GRAVITY)               || \
                                      ((DESC) == MIP_NAV_DATA_FILTER_STATUS)               || \
                                      ((DESC) == MIP_NAV_DATA_FILTER_TIMESTAMP)            || \
                                      ((DESC) == MIP_NAV_DATA_ATT_UNCERTAINTY_QUATERNION)  || \
                                      ((DESC) == MIP_NAV_DATA_GRAVITY_VECTOR)              || \
                                      ((DESC) == MIP_NAV_DATA_HEADING_UPDATE_STATE)        || \
                                      ((DESC) == MIP_NAV_DATA_MAGNETIC_MODEL))


////////////////////////////////////////////////////////////////////////////////
// NAV PARAMETERS
////////////////////////////////////////////////////////////////////////////////

//Bias Estimation Control
#define MIP_NAV_BIAS_ESTIMATION_OFF 0x00
#define MIP_NAV_BIAS_ESTIMATION_ON  0x01


//Dynamics Modes
#define MIP_NAV_DYNAMICS_MODE_PORTABLE    0x01
#define MIP_NAV_DYNAMICS_MODE_AUTOMOTIVE  0x02
#define MIP_NAV_DYNAMICS_MODE_AIRBORNE    0x03


//Heading update sources
#define MIP_NAV_HEADING_SOURCE_NONE          0x00
#define MIP_NAV_HEADING_SOURCE_MAGNETOMETER  0x01
#define MIP_NAV_HEADING_SOURCE_GPS_VELOCITY  0x02
#define MIP_NAV_HEADING_SOURCE_EXTERNAL      0x03

#define IS_NAV_HEADING_SOURCE(SOURCE) (((SOURCE) == MIP_NAV_HEADING_SOURCE_NONE)         || \
                                       ((SOURCE) == MIP_NAV_HEADING_SOURCE_MAGNETOMETER) || \
                                       ((SOURCE) == MIP_NAV_HEADING_SOURCE_GPS_VELOCITY) || \
                                       ((SOURCE) == MIP_NAV_HEADING_SOURCE_EXTERNAL))

//Heading update types
#define MIP_NAV_HEADING_UPDATE_TYPE_TRUE_NORTH      0x01
#define MIP_NAV_HEADING_UPDATE_TYPE_MAGNETIC_NORTH  0x02

#define IS_NAV_HEADING_UPDATE_TYPE(TYPE) (((TYPE) == MIP_NAV_HEADING_UPDATE_TYPE_TRUE_NORTH) || \
                                          ((TYPE) == MIP_NAV_HEADING_UPDATE_TYPE_MAGNETIC_NORTH))
                                          

////////////////////////////////////////////////////////////////////////////////
//
// Flag Definitions
//
////////////////////////////////////////////////////////////////////////////////

///
//EKF Modes
///

#define MIP_NAV_EKF_STATE_STARTUP              0x00
#define MIP_NAV_EKF_STATE_INIT                 0x01
#define MIP_NAV_EKF_STATE_RUN_SOLUTION_VALID   0x02
#define MIP_NAV_EKF_STATE_RUN_SOLUTION_ERROR   0x03

///
//Dynamics Modes
///

#define MIP_NAV_EKF_DYNAMICS_MODE_PORTABLE    0x01
#define MIP_NAV_EKF_DYNAMICS_MODE_AUTOMOTIVE  0x02
#define MIP_NAV_EKF_DYNAMICS_MODE_AIRBORNE    0x03

#define IS_MIP_NAV_EKF_DYNAMICS_MODE(MODE) (((MODE) == MIP_NAV_EKF_DYNAMICS_MODE_PORTABLE)   || \
                                            ((MODE) == MIP_NAV_EKF_DYNAMICS_MODE_AUTOMOTIVE) || \
                                            ((MODE) == MIP_NAV_EKF_DYNAMICS_MODE_AIRBORNE))

///
//EKF Status Flags
///

#define MIP_NAV_EKF_STATUS_FLAG_INIT_NO_ATTITUDE            0x1000
#define MIP_NAV_EKF_STATUS_FLAG_INIT_NO_POSITION_VELOCITY   0x2000

#define MIP_NAV_EKF_STATUS_FLAG_IMU_UNAVAILABLE             0x0001
#define MIP_NAV_EKF_STATUS_FLAG_GPS_UNAVAILABLE             0x0002
#define MIP_NAV_EKF_STATUS_FLAG_MATRIX_SINGULARITY          0x0008
#define MIP_NAV_EKF_STATUS_FLAG_POSITION_COVARIANCE_WARNING 0x0010
#define MIP_NAV_EKF_STATUS_FLAG_VELOCITY_COVARIANCE_WARNING 0x0020
#define MIP_NAV_EKF_STATUS_FLAG_ATTITUDE_COVARIANCE_WARNING 0x0040
#define MIP_NAV_EKF_STATUS_FLAG_NAN_IN_SOLUTION_WARNING     0x0080


///
//Bias Estimation Control
///

#define MIP_NAV_EKF_BIAS_ESTIMATION_CONTROL_OFF 0x00
#define MIP_NAV_EKF_BIAS_ESTIMATION_CONTROL_ON  0x01

#define IS_MIP_NAV_EKF_BIAS_ESTIMATION_CONTROL(CONTROL) (((CONTROL)  == MIP_NAV_EKF_BIAS_ESTIMATION_CONTROL_OFF) || \
                                                         ((CONTROL)  == MIP_NAV_EKF_BIAS_ESTIMATION_CONTROL_ON))


///
// Heading Update State
///

#define NAV_HEADING_UPDATE_STATE_SOURCE_VALID         0x0001
#define NAV_HEADING_UPDATE_STATE_HEADING_VALID        0x0002
#define NAV_HEADING_UPDATE_STATE_HEADING_1SIGMA_VALID 0x0004

#define NAV_HEADING_UPDATE_STATE_PACKET_VALID (NAV_HEADING_UPDATE_STATE_SOURCE_VALID | NAV_HEADING_UPDATE_STATE_HEADING_VALID | NAV_HEADING_UPDATE_STATE_HEADING_1SIGMA_VALID)






////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////

#pragma pack(1)

////////////////////////////////////////////////////////////////////////////////
//
// Commands
//
////////////////////////////////////////////////////////////////////////////////

///
// External GPS Update Information: GPS Time, Position, Velocity
///

typedef struct _mip_nav_external_gps_update_command
{
 double tow;                //(sec)
 u16    week_number; 
 
 double pos[3];             //Lat, Lon, Height (LLH) Position (deg, deg, m)
 float  vel[3];             //NED Velocity, (m/s)

 float  pos_1sigma[3];      //NED position 1-sigma (m)
 float  vel_1sigma[3];      //NED velocity 1-sigma (m/s)
 
}mip_nav_external_gps_update_command;


///
// External Heading Update Information: Heading angle
///

typedef struct _mip_nav_external_heading_update_command
{
 float heading_angle;        //(deg, +-180)
 float heading_angle_1sigma; //(deg)
 u8    type;                 // 1 - true north, 2 - magnetic north
}mip_nav_external_heading_update_command;



////////////////////////////////////////////////////////////////////////////////
//
// Data
//
////////////////////////////////////////////////////////////////////////////////


//LLH Position
typedef struct _mip_nav_llh_pos
{
 double latitude, longitude; //Degrees
 double ellipsoid_height;    //meters
 u16    valid_flags;   
}mip_nav_llh_pos;



//NED Velocity
typedef struct _mip_nav_ned_velocity
{
 float north, east, down;   //meters/sec
 u16   valid_flags;
}mip_nav_ned_velocity;


//Attitude (quaternion)
typedef struct _mip_nav_attitude_quaternion
{
 float q[4];
 u16   valid_flags;
}mip_nav_attitude_quaternion;


//Attitude (DCM)
typedef struct _mip_nav_attitude_dcm
{
 float dcm[3][3];         //Rows, Columns 
 u16   valid_flags;
}mip_nav_attitude_dcm;


//Attitude (Euler Angles)
typedef struct _mip_nav_attitude_euler_angles
{
 float roll, pitch, yaw; //radians
 u16   valid_flags;
}mip_nav_attitude_euler_angles;


//Gyro Bias Estimates
typedef struct _mip_nav_gyro_bias
{
 float x, y, z;    //sensor body frame (radians/sec)
 u16   valid_flags; 
}mip_nav_gyro_bias;


//Accel Bias Estimates
typedef struct _mip_nav_accel_bias
{
 float x, y, z;    //sensor body frame (m/s^2)
 u16   valid_flags; 
}mip_nav_accel_bias;


//LLH Position Uncertainty
typedef struct _mip_nav_llh_pos_uncertainty
{
 float north, east, down;           //1-sigma (meters)
 u16   valid_flags; 
}mip_nav_llh_pos_uncertainty;


//NED Velocity Uncertainty
typedef struct _mip_nav_ned_vel_uncertainty
{
 float north, east, down;           //1-sigma (meters/sec)
 u16   valid_flags; 
}mip_nav_ned_vel_uncertainty;


//Attitude Uncertainty (Euler Angles)
typedef struct _mip_nav_euler_attitude_uncertainty
{
 float roll, pitch, yaw;  //1-sigma (radians)
 u16   valid_flags; 
}mip_nav_euler_attitude_uncertainty;


//Gyro Bias Uncertainty
typedef struct _mip_nav_gyro_bias_uncertainty
{
 float x, y, z;    //sensor body frame (radians/sec)
 u16   valid_flags; 
}mip_nav_gyro_bias_uncertainty;


//Accel Bias Uncertainty
typedef struct _mip_nav_accel_bias_uncertainty
{
 float x, y, z;    //sensor body frame (m/s^2)
 u16   valid_flags; 
}mip_nav_accel_bias_uncertainty;


//Navigation Solution Timetamp
typedef struct _mip_nav_timestamp
{
 double tow;  //Time of Week (seconds)
 u16    week_number; 
 u16    valid_flags; 
}mip_nav_timestamp;


//Navigation Status
typedef struct _mip_nav_status
{
 u16 filter_state;
 u16 dynamics_mode;
 u16 status_flags; 
}mip_nav_status;


//Nav Acceleration Estimate
typedef struct _mip_nav_acceleration 
{
 float x, y, z;   //sensor or vehicle frame (m/s^2)
 u16   valid_flags;
}mip_nav_acceleration;


//Nav Gravity Estimate
typedef struct _mip_nav_gravity_vector 
{
 float x, y, z;   //sensor or vehicle frame (m/s^2)
 u16   valid_flags;
}mip_nav_gravity_vector;


//Nav Angular Rate Estimate
typedef struct _mip_nav_angular_rate 
{
 float x, y, z;   //sensor or vehicle frame (rad/s)
 u16   valid_flags;
}mip_nav_angular_rate;


//Attitude Uncertainty (Quaternion Elements)
typedef struct _mip_nav_quaternion_attitude_uncertainty
{
 float q0, q1, q2, q3;  //1-sigma (radians)
 u16   valid_flags; 
}mip_nav_quaternion_attitude_uncertainty;


//WGS84 Gravity magnitude
typedef struct _mip_nav_wgs84_gravity_mag
{
 float magnitude;  //m/s^2
 u16   valid_flags; 
}mip_nav_wgs84_gravity_mag;


//Heading Update Source
typedef struct _mip_nav_heading_update_state
{
 float heading;
 float heading_1sigma;
 u16   source;
 u16   valid_flags; 
}mip_nav_heading_update_state;


//Magnetic Model
typedef struct _mip_nav_magnetic_model
{
 float intensity_north, intensity_east, intensity_down; //nT 
 float inclination, declination;                        //deg (AKA Dip, Magnetic Variation)
 u16   valid_flags; 
}mip_nav_magnetic_model;


#pragma pack()





////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////


///
//NAV Data
///

void mip_nav_llh_pos_byteswap(mip_nav_llh_pos *llh_pos);
void mip_nav_ned_velocity_byteswap(mip_nav_ned_velocity *ned_velocity);
void mip_nav_attitude_quaternion_byteswap(mip_nav_attitude_quaternion *attitude_quaternion);
void mip_nav_attitude_dcm_byteswap(mip_nav_attitude_dcm *attitude_dcm);
void mip_nav_attitude_euler_angles_byteswap(mip_nav_attitude_euler_angles *attitude_euler_angles);
void mip_nav_gyro_bias_byteswap(mip_nav_gyro_bias *gyro_bias);
void mip_nav_accel_bias_byteswap(mip_nav_accel_bias *accel_bias);
void mip_nav_llh_pos_uncertainty_byteswap(mip_nav_llh_pos_uncertainty *llh_pos_uncertainty);
void mip_nav_ned_vel_uncertainty_byteswap(mip_nav_ned_vel_uncertainty *ned_vel_uncertainty);
void mip_nav_euler_attitude_uncertainty_byteswap(mip_nav_euler_attitude_uncertainty *euler_attitude_uncertainty);
void mip_nav_gyro_bias_uncertainty_byteswap(mip_nav_gyro_bias_uncertainty *gyro_bias_uncertainty);
void mip_nav_accel_bias_uncertainty_byteswap(mip_nav_accel_bias_uncertainty *accel_bias_uncertainty);
void mip_nav_timestamp_byteswap(mip_nav_timestamp *timestamp);
void mip_nav_status_byteswap(mip_nav_status *status);
void mip_nav_acceleration_byteswap(mip_nav_acceleration *acceleration);
void mip_nav_gravity_vector_byteswap(mip_nav_gravity_vector *gravity_vector);
void mip_nav_angular_rate_byteswap(mip_nav_angular_rate *angular_rate);
void mip_nav_quaternion_attitude_uncertainty_byteswap(mip_nav_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty);
void mip_nav_wgs84_gravity_mag_byteswap(mip_nav_wgs84_gravity_mag *wgs84_gravity_mag);
void mip_nav_heading_update_state_byteswap(mip_nav_heading_update_state *heading_update_state);
void mip_nav_magnetic_model_byteswap(mip_nav_magnetic_model *magnetic_model);


///
//NAV Commands
///

u16 mip_nav_reset_filter(mip_interface *device_interface);
u16 mip_nav_set_init_attitude(mip_interface *device_interface, float euler_angles[3]);
u16 mip_nav_set_init_heading(mip_interface *device_interface, float heading);
u16 mip_nav_set_init_attitude_from_ahrs(mip_interface *device_interface, float declination);
u16 mip_nav_vehicle_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode);
u16 mip_nav_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3]);
u16 mip_nav_sensor2vehicle_offset(mip_interface *device_interface, u8 function_selector, float offset[3]);
u16 mip_nav_antenna_offset(mip_interface *device_interface, u8 function_selector, float offset[3]);
u16 mip_nav_bias_estimation(mip_interface *device_interface, u8 function_selector, u16 *bias_control);
u16 mip_nav_gps_source(mip_interface *device_interface, u8 function_selector, u8 *gps_source);
u16 mip_nav_external_gps_update(mip_interface *device_interface, mip_nav_external_gps_update_command *command);
u16 mip_nav_external_heading_update(mip_interface *device_interface, mip_nav_external_heading_update_command *command);
u16 mip_nav_heading_source(mip_interface *device_interface, u8 function_selector, u8 *heading_source);
u16 mip_nav_auto_initialization(mip_interface *device_interface, u8 function_selector, u8 *enable);
u16 mip_nav_accel_white_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3]);
u16 mip_nav_gyro_white_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3]);
u16 mip_nav_gyro_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3]);


#endif