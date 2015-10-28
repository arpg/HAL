/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_nav.c 
//! @author  Nathan Miller
//! @version 1.0
//
//! @description MIP NAV Descriptor Set Definitions
//
// External dependencies:
//
//  mip.h
// 
//! @copyright 2011 Microstrain. 
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

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////


#include "mip_sdk_nav.h"
#include "mip_sdk_system.h"
#include "mip_sdk_user_functions.h"
#include "byteswap_utilities.h"





/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_llh_pos_byteswap(mip_nav_llh_pos *llh_pos)
//
//! @section DESCRIPTION
//! Byteswap a NAV LLH Position Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_llh_pos *llh_pos - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_llh_pos_byteswap(mip_nav_llh_pos *llh_pos)
{
 byteswap_inplace(&llh_pos->latitude,         sizeof(double));
 byteswap_inplace(&llh_pos->longitude,        sizeof(double));
 byteswap_inplace(&llh_pos->ellipsoid_height, sizeof(double));
 byteswap_inplace(&llh_pos->valid_flags,      sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_ned_velocity_byteswap(mip_nav_ned_velocity *ned_velocity)
//
//! @section DESCRIPTION
//! Byteswap a NAV NED Velocity Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_ned_velocity *ned_velocity - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_ned_velocity_byteswap(mip_nav_ned_velocity *ned_velocity)
{
 byteswap_inplace(&ned_velocity->north,       sizeof(float));
 byteswap_inplace(&ned_velocity->east,        sizeof(float));
 byteswap_inplace(&ned_velocity->down,        sizeof(float));
 byteswap_inplace(&ned_velocity->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_attitude_quaternion_byteswap(mip_nav_attitude_quaternion *attitude_quaternion)
//
//! @section DESCRIPTION
//! Byteswap a NAV Quaternion Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_attitude_quaternion *attitude_quaternion - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_attitude_quaternion_byteswap(mip_nav_attitude_quaternion *attitude_quaternion)
{
 u8 i;

 for(i=0; i<4; i++)
  byteswap_inplace(&attitude_quaternion->q[i],       sizeof(float));
 
 byteswap_inplace(&attitude_quaternion->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_attitude_dcm_byteswap(mip_nav_attitude_dcm *attitude_dcm)
//
//! @section DESCRIPTION
//! Byteswap a NAV Direction Cosine Matrix (DCM) Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_attitude_dcm *attitude_dcm - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_attitude_dcm_byteswap(mip_nav_attitude_dcm *attitude_dcm)
{
 u8 i, j;

 for(i=0; i<3; i++)
 {
  for(j=0; j<3; j++)
   byteswap_inplace(&attitude_dcm->dcm[i][j], sizeof(float));
 }
 
 byteswap_inplace(&attitude_dcm->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_attitude_euler_angles_byteswap(mip_nav_attitude_euler_angles *attitude_euler_angles)
//
//! @section DESCRIPTION
//! Byteswap a NAV Euler Angle Attitude Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_attitude_euler_angles *attitude_euler_angles - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_attitude_euler_angles_byteswap(mip_nav_attitude_euler_angles *attitude_euler_angles)
{
 byteswap_inplace(&attitude_euler_angles->roll,        sizeof(float));
 byteswap_inplace(&attitude_euler_angles->pitch,       sizeof(float));
 byteswap_inplace(&attitude_euler_angles->yaw,         sizeof(float));
 byteswap_inplace(&attitude_euler_angles->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_gyro_bias_byteswap(mip_nav_gyro_bias *gyro_bias)
//
//! @section DESCRIPTION
//! Byteswap a NAV Gyro Bias Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_gyro_bias *gyro_bias - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_gyro_bias_byteswap(mip_nav_gyro_bias *gyro_bias)
{
 byteswap_inplace(&gyro_bias->x,           sizeof(float));
 byteswap_inplace(&gyro_bias->y,           sizeof(float));
 byteswap_inplace(&gyro_bias->z,           sizeof(float));
 byteswap_inplace(&gyro_bias->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_accel_bias_byteswap(mip_nav_accel_bias *accel_bias)
//
//! @section DESCRIPTION
//! Byteswap a NAV Accel Bias Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_accel_bias *accel_bias - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_accel_bias_byteswap(mip_nav_accel_bias *accel_bias)
{
 byteswap_inplace(&accel_bias->x,           sizeof(float));
 byteswap_inplace(&accel_bias->y,           sizeof(float));
 byteswap_inplace(&accel_bias->z,           sizeof(float));
 byteswap_inplace(&accel_bias->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_llh_pos_uncertainty_byteswap(mip_nav_llh_pos_uncertainty *llh_pos_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a NAV LLH Position Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_llh_pos_uncertainty *llh_pos_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_llh_pos_uncertainty_byteswap(mip_nav_llh_pos_uncertainty *llh_pos_uncertainty)
{
 byteswap_inplace(&llh_pos_uncertainty->north,       sizeof(float));
 byteswap_inplace(&llh_pos_uncertainty->east,        sizeof(float));
 byteswap_inplace(&llh_pos_uncertainty->down,        sizeof(float));
 byteswap_inplace(&llh_pos_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_ned_vel_uncertainty_byteswap(mip_nav_ned_vel_uncertainty *ned_vel_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a NAV NED Velocity Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_ned_vel_uncertainty *ned_vel_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_ned_vel_uncertainty_byteswap(mip_nav_ned_vel_uncertainty *ned_vel_uncertainty)
{
 byteswap_inplace(&ned_vel_uncertainty->north,       sizeof(float));
 byteswap_inplace(&ned_vel_uncertainty->east,        sizeof(float));
 byteswap_inplace(&ned_vel_uncertainty->down,        sizeof(float));
 byteswap_inplace(&ned_vel_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_euler_attitude_uncertainty_byteswap(mip_nav_euler_attitude_uncertainty *euler_attitude_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a NAV Euler Attitude Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_euler_attitude_uncertainty *euler_attitude_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_euler_attitude_uncertainty_byteswap(mip_nav_euler_attitude_uncertainty *euler_attitude_uncertainty)
{
 byteswap_inplace(&euler_attitude_uncertainty->roll,        sizeof(float));
 byteswap_inplace(&euler_attitude_uncertainty->pitch,       sizeof(float));
 byteswap_inplace(&euler_attitude_uncertainty->yaw,         sizeof(float));
 byteswap_inplace(&euler_attitude_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_gyro_bias_uncertainty_byteswap(mip_nav_gyro_bias_uncertainty *gyro_bias_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a NAV Gyro Bias Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_gyro_bias_uncertainty *gyro_bias_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_gyro_bias_uncertainty_byteswap(mip_nav_gyro_bias_uncertainty *gyro_bias_uncertainty)
{
 byteswap_inplace(&gyro_bias_uncertainty->x,           sizeof(float));
 byteswap_inplace(&gyro_bias_uncertainty->y,           sizeof(float));
 byteswap_inplace(&gyro_bias_uncertainty->z,           sizeof(float));
 byteswap_inplace(&gyro_bias_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_accel_bias_uncertainty_byteswap(mip_nav_accel_bias_uncertainty *accel_bias_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a NAV Accel Bias Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_accel_bias_uncertainty *accel_bias_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_accel_bias_uncertainty_byteswap(mip_nav_accel_bias_uncertainty *accel_bias_uncertainty)
{
 byteswap_inplace(&accel_bias_uncertainty->x,           sizeof(float));
 byteswap_inplace(&accel_bias_uncertainty->y,           sizeof(float));
 byteswap_inplace(&accel_bias_uncertainty->z,           sizeof(float));
 byteswap_inplace(&accel_bias_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_timestamp_byteswap(mip_nav_timestamp *timestamp)
//
//! @section DESCRIPTION
//! Byteswap a NAV Timestamp Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_timestamp *timestamp - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_timestamp_byteswap(mip_nav_timestamp *timestamp)
{
 byteswap_inplace(&timestamp->tow,         sizeof(double));
 byteswap_inplace(&timestamp->week_number, sizeof(u16));
 byteswap_inplace(&timestamp->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_status_byteswap(mip_nav_status *status)
//
//! @section DESCRIPTION
//! Byteswap a NAV Status Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_status *status - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_status_byteswap(mip_nav_status *status)
{
 byteswap_inplace(&status->filter_state,  sizeof(u16));
 byteswap_inplace(&status->dynamics_mode, sizeof(u16));
 byteswap_inplace(&status->status_flags,  sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_acceleration_byteswap(mip_nav_acceleration *acceleration)
//
//! @section DESCRIPTION
//! Byteswap a NAV Acceleration Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_acceleration *acceleration - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_acceleration_byteswap(mip_nav_acceleration *acceleration)
{
 byteswap_inplace(&acceleration->x,           sizeof(float));
 byteswap_inplace(&acceleration->y,           sizeof(float));
 byteswap_inplace(&acceleration->z,           sizeof(float));
 byteswap_inplace(&acceleration->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_gravity_vector_byteswap(mip_nav_gravity_vector *gravity_vector)
//
//! @section DESCRIPTION
//! Byteswap a NAV Gravity Vector Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_gravity_vector *gravity_vector - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_gravity_vector_byteswap(mip_nav_gravity_vector *gravity_vector)
{
 byteswap_inplace(&gravity_vector->x,           sizeof(float));
 byteswap_inplace(&gravity_vector->y,           sizeof(float));
 byteswap_inplace(&gravity_vector->z,           sizeof(float));
 byteswap_inplace(&gravity_vector->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_angular_rate_byteswap(mip_nav_angular_rate *angular_rate)
//
//! @section DESCRIPTION
//! Byteswap a NAV Angular Rate Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_angular_rate *angular_rate - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_angular_rate_byteswap(mip_nav_angular_rate *angular_rate)
{
 byteswap_inplace(&angular_rate->x,           sizeof(float));
 byteswap_inplace(&angular_rate->y,           sizeof(float));
 byteswap_inplace(&angular_rate->z,           sizeof(float));
 byteswap_inplace(&angular_rate->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_quaternion_attitude_uncertainty_byteswap(mip_nav_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty)
//
//! @section DESCRIPTION
//! Byteswap a NAV Quaternion Attitude Uncertainty Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_quaternion_attitude_uncertainty_byteswap(mip_nav_quaternion_attitude_uncertainty *quaternion_attitude_uncertainty)
{
 byteswap_inplace(&quaternion_attitude_uncertainty->q0,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->q1,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->q2,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->q3,          sizeof(float));
 byteswap_inplace(&quaternion_attitude_uncertainty->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_wgs84_gravity_mag_byteswap(mip_nav_wgs84_gravity_mag *wgs84_gravity_mag)
//
//! @section DESCRIPTION
//! Byteswap a NAV WGS84 Gravity Magnitude Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_wgs84_gravity_mag *wgs84_gravity_mag - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_wgs84_gravity_mag_byteswap(mip_nav_wgs84_gravity_mag *wgs84_gravity_mag)
{
 byteswap_inplace(&wgs84_gravity_mag->magnitude,   sizeof(float));
 byteswap_inplace(&wgs84_gravity_mag->valid_flags, sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_heading_update_state_byteswap(mip_nav_heading_update_state *heading_update_state)
//
//! @section DESCRIPTION
//! Byteswap a NAV Heading Update State Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_heading_update_state *heading_update_state - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_heading_update_state_byteswap(mip_nav_heading_update_state *heading_update_state)
{
 byteswap_inplace(&heading_update_state->heading,        sizeof(float));
 byteswap_inplace(&heading_update_state->heading_1sigma, sizeof(float));
 byteswap_inplace(&heading_update_state->source,         sizeof(u16));
 byteswap_inplace(&heading_update_state->valid_flags,    sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! void mip_nav_magnetic_model_byteswap(mip_nav_magnetic_model *magnetic_model)
//
//! @section DESCRIPTION
//! Byteswap a NAV Magnetic Model Structure.  
//
//! @section DETAILS
//!
//! @param [in] mip_nav_magnetic_model *magnetic_model - The structure to be byteswapped.
//
//! @section NOTES
//! 
//! None
//
/////////////////////////////////////////////////////////////////////////////

void mip_nav_magnetic_model_byteswap(mip_nav_magnetic_model *magnetic_model)
{
 byteswap_inplace(&magnetic_model->intensity_north, sizeof(float));
 byteswap_inplace(&magnetic_model->intensity_east,  sizeof(float));
 byteswap_inplace(&magnetic_model->intensity_down,  sizeof(float));
 byteswap_inplace(&magnetic_model->inclination,     sizeof(float));
 byteswap_inplace(&magnetic_model->declination,     sizeof(float));
 byteswap_inplace(&magnetic_model->valid_flags,     sizeof(u16));
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_reset_filter(mip_interface *device_interface)
//
//! @section DESCRIPTION
//! Reset the Kalman Filter.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_reset_filter(mip_interface *device_interface)
{
 return mip_interface_send_command(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_RESET_FILTER, 
                                   NULL, 0, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_set_init_attitude(mip_interface *device_interface, float euler_angles[3])
//
//! @section DESCRIPTION
//! Initialize the Kalman Filter with the provided Euler angles.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] float euler_angles[3]           - The Euler angles in radians.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Order of angles is [roll, pitch, yaw] in radians.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_set_init_attitude(mip_interface *device_interface, float euler_angles[3])
{
 u8 i;
 float local_angles[3];
 
 //Copy the angles to a local buffer
 memcpy(local_angles, euler_angles, sizeof(float)*3);
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  for(i=0; i<3; i++)
   byteswap_inplace(&local_angles[i], sizeof(float));
 }   


 return mip_interface_send_command(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_SET_INITIAL_ATTITUDE, 
                                   (u8*)local_angles, sizeof(float)*3, 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_set_init_heading(mip_interface *device_interface, float heading)
//
//! @section DESCRIPTION
//! Initialize the Kalman Filter with the provided true heading angle.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] float heading                   - The true heading angle in radians.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! Roll and Pitch will be calculated by the device using the accelerometers.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_set_init_heading(mip_interface *device_interface, float heading)
{
// u8 i;
 float local_heading;
 
 //Copy the angles to a local buffer
 memcpy(&local_heading, &heading, sizeof(float));
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_heading, sizeof(float));
 }   


 return mip_interface_send_command(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_SET_INITIAL_HEADING, 
                                   (u8*)&local_heading, sizeof(float), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_set_init_attitude_from_ahrs(mip_interface *device_interface, float declination)
//
//! @section DESCRIPTION
//! Initialize the Kalman Filter from the AHRS algorithm output, taking into account the magnetic declination angle povided.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] float declination               - The local magnetic declination angle in radians.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! The output of the internal AHRS orientation algorithm will be used to initialize\n
//! the filter, taking into account the user-provided magnetic declination angle.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_set_init_attitude_from_ahrs(mip_interface *device_interface, float declination)
{
// u8 i;
 float local_declination;
 
 //Copy the angles to a local buffer
 memcpy(&local_declination, &declination, sizeof(float));
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_declination, sizeof(float));
 }   


 return mip_interface_send_command(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_SET_INITIAL_HEADING_FROM_AHRS, 
                                   (u8*)&local_declination, sizeof(float), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);

}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_vehicle_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode)
//
//! @section DESCRIPTION
//! Set or read the filter's vehicle dynamics mode settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface        - The device interface.
//! @param [in] u8 function_selector                   - Selects which function to perform.
//! @param [in,out] u8 *dynamics_mode                   - The vehicle dynamics mode. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! Please reference the device DCP for valid \c dynamics_mode values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_vehicle_dynamics_mode(mip_interface *device_interface, u8 function_selector, u8 *dynamics_mode)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *dynamics_mode;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_VEHICLE_DYNAMICS_MODE, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_VEHICLE_DYNAMICS_MODE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(dynamics_mode, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3])
//
//! @section DESCRIPTION
//! Set or read the filter's sensor to vehicle transformation settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface        - The device interface.
//! @param [in] u8 function_selector                   - Selects which function to perform.
//! @param [in,out] float euler_angles[3]              - The sensor to vehicle transformation expressed as Euler angles\n
//!                                                      in radians. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The angles are [roll, pitch, yaw] in radians from the sensor frame to the vehicle frame.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_sensor2vehicle_tranformation(mip_interface *device_interface, u8 function_selector, float euler_angles[3])
{
 u8 i;
 u8 *response_data      = NULL;
 u16 response_data_size = 0;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr = (float*)&command_data[1];

  //Copy the angles to a local buffer
  memcpy(float_ptr, euler_angles, sizeof(float)*3);
 
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }  
 }
 

                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_SENSOR2VEHICLE_TRANSFORMATION, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_SENSOR2VEHICLE_TRANSFORMATION) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(euler_angles, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&euler_angles[i], sizeof(float));
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;                                   
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_sensor2vehicle_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
//
//! @section DESCRIPTION
//! Set or read the filter's sensor to vehicle frame offset settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float offset                - The sensor to vehicle frame offset in meters. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The offset is [x, y, z] in meters from the sensor frame to the vehicle frame, expressed in the sensor frame.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_sensor2vehicle_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3]={0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
     float_ptr = (float*)&command_data[1];
  
  //Copy the angles to a local buffer
  memcpy(float_ptr, offset, sizeof(float)*3);
 
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
 
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_SENSOR2VEHICLE_OFFSET, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                                                     
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_SENSOR2VEHICLE_OFFSET) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(offset, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&offset[i], sizeof(float));
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_antenna_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
//
//! @section DESCRIPTION
//! Set or read the filter's antenna offset settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float offset                - The offset of the antenna with-respect-to the sensor in meters. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The offset is [x, y, z] in meters from the sensor to the antenna, expressed in the sensor frame.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_antenna_offset(mip_interface *device_interface, u8 function_selector, float offset[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3]={0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;

 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, offset, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  { 
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }
 }   
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_ANTENNA_OFFSET, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                   
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_ANTENNA_OFFSET) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(offset, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&offset[i], sizeof(float));
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_bias_estimation(mip_interface *device_interface, u8 function_selector, u8 *bias_control)
//
//! @section DESCRIPTION
//! Set or read the filter's bias estimation control settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u16 *bias_control           - The bais control value. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! Please consult the device DCP for valid bias control values
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_bias_estimation(mip_interface *device_interface, u8 function_selector, u16 *bias_control)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[3] = {0};
 mip_field_header *field_header_ptr;
 u16 *short_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;

 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  short_ptr       = (u16*)&command_data[1];
  *short_ptr      = *bias_control;

  //Byteswap the bias control value if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   byteswap_inplace(short_ptr, sizeof(u16));
  }   
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_BIAS_ESTIMATION_CONTROL, command_data, 
                                                        3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_BIAS_ESTIMATION_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(bias_control, response_data + sizeof(mip_field_header), sizeof(u16));
   
   //Byteswap the bias control value if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    byteswap_inplace(bias_control, sizeof(u16));
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_gps_source(mip_interface *device_interface, u8 function_selector, u8 *gps_source)
//
//! @section DESCRIPTION
//! Set or read the filter's GPS source settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *gps_source              - The source of GPS updates to the filter. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! Please consult the device DCP for valid GPS source values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_gps_source(mip_interface *device_interface, u8 function_selector, u8 *gps_source)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2];
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *gps_source;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_GPS_SOURCE_CONTROL, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_GPS_SOURCE_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(gps_source, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_external_gps_update(mip_interface *device_interface, mip_nav_external_gps_update_command *command)
//
//! @section DESCRIPTION
//! Provide an external GPS update to the filter.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface              - The device interface.
//! @param [in] mip_nav_external_gps_update_command *command - The external GPS update command.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//!
//! The device will only accept this command when external GPS is selected\n
//! as the GPS source.  Please consult the device DCP for how to configure this setting.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_external_gps_update(mip_interface *device_interface, mip_nav_external_gps_update_command *command)
{
 u8 i;
 mip_nav_external_gps_update_command local_command;
 
 //Copy the command to the local buffer
 memcpy(&local_command, command, sizeof(mip_nav_external_gps_update_command));
 
 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_command.tow,            sizeof(double));
  byteswap_inplace(&local_command.week_number,    sizeof(u16));

  for(i=0; i<3; i++)
  {
   byteswap_inplace(&local_command.pos[i],        sizeof(double));
   byteswap_inplace(&local_command.vel[i],        sizeof(float));
   byteswap_inplace(&local_command.pos_1sigma[i], sizeof(float));
   byteswap_inplace(&local_command.vel_1sigma[i], sizeof(float));
  }
 }   
 
 return mip_interface_send_command(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_EXTERNAL_GPS_UPDATE, 
                                   (u8*)&local_command, sizeof(mip_nav_external_gps_update_command), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_external_heading_update(mip_interface *device_interface, mip_nav_external_heading_update_command *command)
//
//! @section DESCRIPTION
//! Provide an external heading update to the filter.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface                  - The device interface.
//! @param [in] mip_nav_external_heading_update_command *command - The external heading update command.
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//!
//! The device will only accept this command when external heading is selected\n
//! as the heading update source.  Please consult the device DCP for how to configure this setting.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_external_heading_update(mip_interface *device_interface, mip_nav_external_heading_update_command *command)
{
// u8 i;
 mip_nav_external_heading_update_command local_command;
 
 //Copy the command to the local buffer
 memcpy(&local_command, command, sizeof(mip_nav_external_heading_update_command));

 //Byteswap the angles if enabled
 if(MIP_SDK_CONFIG_BYTESWAP)
 {
  byteswap_inplace(&local_command.heading_angle,           sizeof(float));
  byteswap_inplace(&local_command.heading_angle_1sigma,    sizeof(float));   
 }   
 

 return mip_interface_send_command(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_EXTERNAL_HEADING_UPDATE, 
                                   (u8*)&local_command, sizeof(mip_nav_external_heading_update_command), 1, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_heading_source(mip_interface *device_interface, u8 function_selector, u8 *heading_source)
//
//! @section DESCRIPTION
//! Set or read the filter's heading update source settings.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *heading_source          - The source of heading updates to the filter. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! Please consult the device DCP for valid heading update source values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_heading_source(mip_interface *device_interface, u8 function_selector, u8 *heading_source)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *heading_source;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_HEADING_UPDATE_CONTROL, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_HEADING_UPDATE_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(heading_source, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_auto_initialization(mip_interface *device_interface, u8 function_selector, u8 *enable)
//
//! @section DESCRIPTION
//! Set or read the filter's auto-initialization setting.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] u8 *enable                  - The auto-initialization setting. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! Please consult the device DCP for valid auto-initialization values.
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_auto_initialization(mip_interface *device_interface, u8 function_selector, u8 *enable)
{
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[2] = {0};
 mip_field_header *field_header_ptr;
 
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  command_data[1] = *enable;
 }
 
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_AUTOINIT_CONTROL, command_data, 
                                                        2, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
 
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_AUTOINIT_CONTROL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(u8)))
  {
   memcpy(enable, response_data + sizeof(mip_field_header), sizeof(u8));
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_accel_white_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's accelerometer white noise values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float noise_1sigma[3]       - The accelerometer 1-sigma white noise values in m/(s^2). (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The accelerometer 1-sigma white noise values are ordered [x, y, z] in the\n
//! sensor frame in m/(s^2).  These values are used as process noise and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_accel_white_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
  
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_ACCEL_NOISE, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_ACCEL_NOISE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(noise_1sigma, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&noise_1sigma[i], sizeof(float));
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_gyro_white_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's gyroscope white noise values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float noise_1sigma[3]       - The gyroscope 1-sigma white noise values in rad/sec. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The gyroscope 1-sigma white noise values are ordered [x, y, z] in the\n
//! sensor frame in rad/sec.  These values are used as process noise and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_gyro_white_noise(mip_interface *device_interface, u8 function_selector, float noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*3] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
 
  //Copy the angles to a local buffer
  memcpy(float_ptr, noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<3; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }    
 }
                                  
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_GYRO_NOISE, command_data, 
                                                        sizeof(u8) + sizeof(float)*3, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_GYRO_NOISE) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*3))
  {
   memcpy(noise_1sigma, response_data + sizeof(mip_field_header), sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
     byteswap_inplace(&noise_1sigma[i], sizeof(float));
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;  
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_nav_gyro_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3])
//
//! @section DESCRIPTION
//! Set or read the filter's gyroscope Guass-Markov bias model values.  
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - The device interface.
//! @param [in] u8 function_selector            - Selects which function to perform.
//! @param [in,out] float bias_beta[3]          - The gyroscope bias model time constants in 1/sec. (used to set or get depending on \c function_selector)
//! @param [in,out] float bias_noise_1sigma[3]  - The gyroscope bias model 1-sigma white noise values in rad/sec. (used to set or get depending on \c function_selector)
//
//! @retval MIP_INTERFACE_ERROR  When there is a problem with the command format or the\n
//!                              the device communications failed.\n
//! @retval MIP_INTERFACE_OK     The command was successful.\n
//
//! @section NOTES
//! 
//! \n Possible \c function_selector values:\n
//!    \li 0x01 - Use New Settings
//!    \li 0x02 - Read Current Settings
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! \c settings may be NULL for the following \c function_selector values:
//!
//!    \li 0x03 - Save Current Settings as Startup Settings
//!    \li 0x04 - Load Saved Settings
//!    \li 0x05 - Load Factory Default Settings
//!
//! The gyroscope Gauss-Markov bias model time constant values are ordered [x, y, z] in the\n
//! sensor frame in units of 1/sec.  The gyro Gauss-Markov bias model 1-sigma white noise values are ordered [x, y, z] in the\n
//! sensor frame in units of rad/sec.  These values are used as process noise to the gyro bias model and allow the user to tune\n
//! the behavior of the Kalman filter for their particular application.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_nav_gyro_bias_model(mip_interface *device_interface, u8 function_selector, float bias_beta[3], float bias_noise_1sigma[3])
{
 u8 i;
 u8 *response_data;
 u16 response_data_size;
 u16 return_code;
 u8  command_data[sizeof(u8) + sizeof(float)*6] = {0};
 mip_field_header *field_header_ptr;
 float *float_ptr;
 
 //Fill-in the command data
 command_data[0] = function_selector;
 
 
 if(function_selector == MIP_FUNCTION_SELECTOR_WRITE)
 {
  float_ptr       = (float*)&command_data[1];
  
  //Copy the parameters to a local buffer
  memcpy(float_ptr,   bias_beta,         sizeof(float)*3);
  memcpy(float_ptr+3, bias_noise_1sigma, sizeof(float)*3);
 
  //Byteswap the angles if enabled
  if(MIP_SDK_CONFIG_BYTESWAP)
  {
   for(i=0; i<6; i++)
    byteswap_inplace(&float_ptr[i], sizeof(float));
  }   
 }
                                   
 return_code = mip_interface_send_command_with_response(device_interface, MIP_NAV_COMMAND_SET, MIP_NAV_CMD_GYRO_BIAS_MODEL, command_data, 
                                                        sizeof(u8) + sizeof(float)*6, &response_data, &response_data_size, MIP_INTERFACE_DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS);
                                  
 //Copy the data to the provided buffer on success if present
 if((return_code == MIP_INTERFACE_OK) && (response_data != NULL))
 {
  field_header_ptr = (mip_field_header*)response_data;
  
  if((field_header_ptr->descriptor == MIP_NAV_REPLY_GYRO_BIAS_MODEL) &&
     (field_header_ptr->size >= sizeof(mip_field_header) + sizeof(float)*6))
  {
   float_ptr = (float*)(response_data + sizeof(mip_field_header));
  
   memcpy(bias_beta,         float_ptr,   sizeof(float)*3);
   memcpy(bias_noise_1sigma, float_ptr+3, sizeof(float)*3);
   
   //Byteswap the angles if enabled
   if(MIP_SDK_CONFIG_BYTESWAP)
   {
    for(i=0; i<3; i++)
    {
     byteswap_inplace(&bias_beta[i],         sizeof(float));
     byteswap_inplace(&bias_noise_1sigma[i], sizeof(float));
    }
   }   
  }
  else 
   return_code = MIP_INTERFACE_ERROR;
 }
 
 return return_code;  
}


