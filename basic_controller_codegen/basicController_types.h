//
// File: basicController_types.h
//
// Code generated for Simulink model 'basicController'.
//
// Model version                  : 1.40
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Thu Mar 23 21:40:27 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-A
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_basicController_types_h_
#define RTW_HEADER_basicController_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_e_lpe_s_
#define DEFINED_TYPEDEF_FOR_e_lpe_s_

// Estimation of position
struct e_lpe_s
{
  uint64_T time_stamp;
  real_T pos_ned[3];
  real_T vel_ned[3];
  real_T pos_accel_body[3];
  real_T accel_bias[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_e_cf_s_
#define DEFINED_TYPEDEF_FOR_e_cf_s_

// estimation of attitude Quat
struct e_cf_s
{
  uint64_T time_stamp;
  real_T quat_data[4];
  real_T roll;
  real_T pitch;
  real_T yaw;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_c_subs_s_
#define DEFINED_TYPEDEF_FOR_c_subs_s_

struct c_subs_s
{
  uint64_T time_stamp;
  uint16_T channels[16];
  boolean_T ch17;
  boolean_T ch18;
  boolean_T failsafe;
  boolean_T frameLost;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_m_gyro_s_
#define DEFINED_TYPEDEF_FOR_m_gyro_s_

struct m_gyro_s
{
  uint64_T time_stamp;
  real32_T gyro_data[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_m_accel_s_
#define DEFINED_TYPEDEF_FOR_m_accel_s_

struct m_accel_s
{
  uint64_T time_stamp;
  real32_T accel_data[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_m_mag_s_
#define DEFINED_TYPEDEF_FOR_m_mag_s_

struct m_mag_s
{
  uint64_T time_stamp;
  real32_T mag_data[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_m_gps_s_
#define DEFINED_TYPEDEF_FOR_m_gps_s_

struct m_gps_s
{
  boolean_T updated;
  boolean_T ned_origin_valid;
  boolean_T gps_is_good;
  uint64_T time_stamp;
  real_T lon;
  real_T lat;
  real_T lon_origin;
  real_T lat_origin;
  real32_T yaw_offset;
  real32_T height;
  real32_T vel_ned[3];
  real32_T pos_ned[3];
  real32_T hacc;
  real32_T vacc;
  real32_T sacc;
  real32_T heading;
  real32_T headacc;
  uint8_T numSV;
  uint8_T fixType;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_m_baro_s_
#define DEFINED_TYPEDEF_FOR_m_baro_s_

struct m_baro_s
{
  uint64_T time_stamp;
  real_T pressure;
  real_T temperature;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_c_out_s_
#define DEFINED_TYPEDEF_FOR_c_out_s_

struct c_out_s
{
  uint64_T time_stamp;
  uint16_T pwm[4];
  real32_T thrust[4];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_s_scope_s_
#define DEFINED_TYPEDEF_FOR_s_scope_s_

struct s_scope_s
{
  uint64_T time_stamp;
  uint32_T rate_hz;
  real32_T data[40];
};

#endif
#endif                                 // RTW_HEADER_basicController_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
