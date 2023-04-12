//
// File: PIDControllerCodeGen.cpp
//
// Code generated for Simulink model 'PIDControllerCodeGen'.
//
// Model version                  : 1.2
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Thu Mar 23 21:40:22 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-A
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "PIDControllerCodeGen.h"
#include "rtwtypes.h"
#include <cmath>
#include "rt_atan2f_snf.h"
#include "norm_afMWvhHe.h"

extern "C"
{

#include "rt_nonfinite.h"

}

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         (*((rtm)->errorStatus))
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    (*((rtm)->errorStatus) = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  (rtm)->errorStatus
#endif

#ifndef rtmSetErrorStatusPointer
#define rtmSetErrorStatusPointer(rtm, val) ((rtm)->errorStatus = (val))
#endif

//
// Output and update for atomic system:
//    '<S3>/deadzone1'
//    '<S3>/deadzone2'
//    '<S3>/deadzone3'
//    '<S3>/deadzone4'
//
void PIDControllerCodeGen::PIDControllerCodeGen_deadzone1(real32_T rtu_u,
  real32_T *rty_y)
{
  real32_T u;
  u = rtu_u;
  if (rtu_u < 1100.0F) {
    u = 1100.0F;
  } else if (rtu_u > 1900.0F) {
    u = 1900.0F;
  }

  if (u > 1540.0F) {
    *rty_y = ((u - 1500.0F) - 40.0F) * 0.00277777785F;
  } else if (u < 1460.0F) {
    *rty_y = ((u - 1500.0F) + 40.0F) * 0.00277777785F;
  } else {
    *rty_y = 0.0F;
  }
}

//
// Output and update for atomic system:
//    '<S96>/MATLAB Function2'
//    '<S97>/MATLAB Function2'
//    '<S98>/MATLAB Function2'
//
void PIDControllerCodeGen::PIDControllerCo_MATLABFunction2(real32_T
  rtu_integral_0, real32_T rtu_deltaT, real32_T rtu_u, real32_T rtu_reset,
  real32_T rtu_reset_value, real32_T *rty_integral_t)
{
  *rty_integral_t = rtu_deltaT * rtu_u + rtu_integral_0;
  if (rtu_reset == 1.0F) {
    *rty_integral_t = rtu_reset_value;
  }
}

//
// Output and update for atomic system:
//    '<S105>/MATLAB Function2'
//    '<S107>/MATLAB Function2'
//    '<S109>/MATLAB Function2'
//
void PIDControllerCodeGen::PIDController_MATLABFunction2_i(real32_T
  rtu_integral_0, real32_T rtu_deltaT, real32_T rtu_u, real32_T rtu_max_value,
  real32_T rtu_reset, real32_T *rty_integral_t)
{
  *rty_integral_t = rtu_deltaT * rtu_u + rtu_integral_0;
  if (*rty_integral_t > rtu_max_value) {
    *rty_integral_t = rtu_max_value;
  } else if (*rty_integral_t < -rtu_max_value) {
    *rty_integral_t = -rtu_max_value;
  }

  if (rtu_reset == 1.0F) {
    *rty_integral_t = 0.0F;
  }
}

// System initialize for referenced model: 'PIDControllerCodeGen'
void PIDControllerCodeGen::init(void)
{
  // SystemInitialize for Merge: '<S2>/Merge'
  PIDControllerCodeGenrtDW.Merge[0] = 1.0F;
  PIDControllerCodeGenrtDW.Merge[1] = 0.0F;
  PIDControllerCodeGenrtDW.Merge[2] = 0.0F;
  PIDControllerCodeGenrtDW.Merge[3] = 0.0F;
}

// Output and update for referenced model: 'PIDControllerCodeGen'
void PIDControllerCodeGen::step(const uint64_T *rtu_usec, const real_T
  rtu__e_lpe_s_pos_ned[3], const real_T rtu__e_lpe_s_vel_ned[3], const real_T
  rtu__e_cf_s_quat_data[4], const uint16_T rtu__c_subs_s_channels[16], const
  uint64_T *rtu__m_gyro_s_time_stamp, const real32_T rtu__m_gyro_s_gyro_data[3],
  uint64_T *rty__c_out_s_time_stamp, uint16_T rty__c_out_s_pwm[4], real32_T
  rty__c_out_s_thrust[4], uint64_T *rty__s_scope_s_time_stamp, uint32_T
  *rty__s_scope_s_rate_hz, real32_T rty__s_scope_s_data[40])
{
  real32_T tmp[16];
  real32_T rtb_TmpSignalConversionAtBusC_a[4];
  real32_T rtb_maxvelinvelcontroller[3];
  real32_T rtb_maxvelinvelcontroller_0[3];
  real32_T Yq_idx_1;
  real32_T Yq_idx_2;
  real32_T dcm02;
  real32_T dcm12;
  real32_T rtb_Gain5;
  real32_T rtb_Gain6;
  real32_T rtb_Rd_idx_0;
  real32_T rtb_Rd_idx_3;
  real32_T rtb_Rd_idx_6;
  real32_T rtb_Saturation4_idx_1;
  real32_T rtb_Saturation9;
  real32_T rtb_accel_n_sp_idx_0;
  real32_T rtb_accel_n_sp_idx_1;
  real32_T rtb_accel_n_sp_idx_2;
  real32_T rtb_accel_yaw_idx_0;
  real32_T rtb_accel_yaw_idx_2;
  real32_T rtb_fcn2;
  real32_T rtb_fcn3_k;
  real32_T rtb_integral_t_f;
  real32_T rtb_integral_t_g_tmp;
  real32_T rtb_integral_t_l;
  real32_T rtb_y_i;
  uint16_T rtb_DataTypeConversion[16];
  boolean_T rtb_Gain6_0;

  // Sum: '<S17>/Sum' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   MATLAB Function: '<S3>/quat2eul'
  //   Product: '<S17>/Product'
  //   Product: '<S17>/Product1'

  rtb_Saturation9 = static_cast<real32_T>(rtu__e_cf_s_quat_data[0]) *
    static_cast<real32_T>(rtu__e_cf_s_quat_data[0]) + static_cast<real32_T>
    (rtu__e_cf_s_quat_data[1]) * static_cast<real32_T>(rtu__e_cf_s_quat_data[1]);

  // Product: '<S17>/Product2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   MATLAB Function: '<S3>/quat2eul'

  rtb_accel_yaw_idx_2 = static_cast<real32_T>(rtu__e_cf_s_quat_data[2]) *
    static_cast<real32_T>(rtu__e_cf_s_quat_data[2]);

  // Product: '<S17>/Product3' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   MATLAB Function: '<S3>/quat2eul'

  rtb_Gain6 = static_cast<real32_T>(rtu__e_cf_s_quat_data[3]) *
    static_cast<real32_T>(rtu__e_cf_s_quat_data[3]);

  // Sum: '<S17>/Sum' incorporates:
  //   Product: '<S17>/Product2'
  //   Product: '<S17>/Product3'
  //   Sum: '<S91>/Sum'

  rtb_fcn3_k = (rtb_Saturation9 + rtb_accel_yaw_idx_2) + rtb_Gain6;

  // Product: '<S10>/Divide' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   Sum: '<S17>/Sum'

  rtb_fcn2 = static_cast<real32_T>(rtu__e_cf_s_quat_data[0]) / rtb_fcn3_k;

  // DataTypeConversion: '<S3>/Data Type Conversion'
  for (int32_T i{0}; i < 16; i++) {
    rtb_DataTypeConversion[i] = rtu__c_subs_s_channels[i];
  }

  // End of DataTypeConversion: '<S3>/Data Type Conversion'

  // MATLAB Function: '<S3>/flight_mode'
  if (rtb_DataTypeConversion[4] < 1200) {
    rtb_y_i = 1.0F;
  } else if (rtb_DataTypeConversion[4] > 1800) {
    rtb_y_i = 3.0F;
  } else {
    rtb_y_i = 2.0F;
  }

  // End of MATLAB Function: '<S3>/flight_mode'

  // MATLAB Function: '<S3>/quat2eul' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'

  rtb_accel_n_sp_idx_2 = static_cast<real32_T>(rtu__e_cf_s_quat_data[1]) *
    static_cast<real32_T>(rtu__e_cf_s_quat_data[3]);
  rtb_accel_yaw_idx_0 = static_cast<real32_T>(rtu__e_cf_s_quat_data[0]) *
    static_cast<real32_T>(rtu__e_cf_s_quat_data[2]);
  dcm02 = (rtb_accel_yaw_idx_0 + rtb_accel_n_sp_idx_2) * 2.0F;
  dcm12 = (static_cast<real32_T>(rtu__e_cf_s_quat_data[2]) *
           static_cast<real32_T>(rtu__e_cf_s_quat_data[3]) -
           static_cast<real32_T>(rtu__e_cf_s_quat_data[0]) *
           static_cast<real32_T>(rtu__e_cf_s_quat_data[1])) * 2.0F;
  rtb_integral_t_f = std::asin(-((rtb_accel_n_sp_idx_2 - rtb_accel_yaw_idx_0) *
    2.0F));
  if (std::abs(rtb_integral_t_f - 1.57079637F) < 0.001) {
    dcm02 = rt_atan2f_snf(dcm12, dcm02);
  } else if (std::abs(rtb_integral_t_f + 1.57079637F) < 0.001) {
    dcm02 = rt_atan2f_snf(-dcm12, -dcm02);
  } else {
    dcm02 = rt_atan2f_snf((static_cast<real32_T>(rtu__e_cf_s_quat_data[1]) *
      static_cast<real32_T>(rtu__e_cf_s_quat_data[2]) + static_cast<real32_T>
      (rtu__e_cf_s_quat_data[0]) * static_cast<real32_T>(rtu__e_cf_s_quat_data[3]))
                          * 2.0F, (rtb_Saturation9 - rtb_accel_yaw_idx_2) -
                          rtb_Gain6);
  }

  // MATLAB Function: '<S3>/deadzone1'
  PIDControllerCodeGen_deadzone1(static_cast<real32_T>(rtb_DataTypeConversion[1]),
    &rtb_integral_t_f);

  // Saturate: '<S3>/Saturation8'
  if (rtb_integral_t_f > 1.0F) {
    rtb_accel_yaw_idx_0 = 1.0F;

    // Gain: '<S3>/Gain7'
    dcm12 = -1.0F;
  } else if (rtb_integral_t_f < -1.0F) {
    rtb_accel_yaw_idx_0 = -1.0F;

    // Gain: '<S3>/Gain7'
    dcm12 = 1.0F;
  } else {
    rtb_accel_yaw_idx_0 = rtb_integral_t_f;

    // Gain: '<S3>/Gain7'
    dcm12 = -rtb_integral_t_f;
  }

  // End of Saturate: '<S3>/Saturation8'

  // MATLAB Function: '<S3>/deadzone2'
  PIDControllerCodeGen_deadzone1(static_cast<real32_T>(rtb_DataTypeConversion[0]),
    &rtb_integral_t_l);

  // Saturate: '<S3>/Saturation9'
  if (rtb_integral_t_l > 1.0F) {
    rtb_Saturation9 = 1.0F;
  } else if (rtb_integral_t_l < -1.0F) {
    rtb_Saturation9 = -1.0F;
  } else {
    rtb_Saturation9 = rtb_integral_t_l;
  }

  // End of Saturate: '<S3>/Saturation9'

  // MATLAB Function: '<S3>/deadzone4'
  PIDControllerCodeGen_deadzone1(static_cast<real32_T>(rtb_DataTypeConversion[2]),
    &rtb_integral_t_f);

  // Saturate: '<S3>/Saturation7'
  if (rtb_integral_t_f > 1.0F) {
    rtb_Gain5 = 1.0F;

    // MATLAB Function: '<S3>/MATLAB Function1' incorporates:
    //   Gain: '<S3>/Gain5'

    rtb_accel_yaw_idx_2 = -1.0F;
  } else if (rtb_integral_t_f < -1.0F) {
    rtb_Gain5 = -1.0F;

    // MATLAB Function: '<S3>/MATLAB Function1' incorporates:
    //   Gain: '<S3>/Gain5'

    rtb_accel_yaw_idx_2 = 1.0F;
  } else {
    rtb_Gain5 = rtb_integral_t_f;

    // MATLAB Function: '<S3>/MATLAB Function1' incorporates:
    //   Gain: '<S3>/Gain5'

    rtb_accel_yaw_idx_2 = -rtb_integral_t_f;
  }

  // End of Saturate: '<S3>/Saturation7'

  // Gain: '<S3>/Gain6' incorporates:
  //   Gain: '<S3>/Gain5'

  rtb_Gain6 = 9.0F * -rtb_Gain5;

  // MATLAB Function: '<S3>/MATLAB Function2' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function3'

  rtb_accel_n_sp_idx_2 = rtb_Gain6;

  // MATLAB Function: '<S3>/MATLAB Function3' incorporates:
  //   Gain: '<S3>/Gain10'
  //   Gain: '<S3>/Gain11'
  //   Gain: '<S3>/Gain7'

  rtb_accel_yaw_idx_0 = (rtb_Gain6 - 9.81F) * std::tan(-0.52359879F *
    -rtb_accel_yaw_idx_0);
  rtb_integral_t_f = (rtb_Gain6 - 9.81F) * std::tan(-0.52359879F *
    rtb_Saturation9);

  // MATLAB Function: '<S3>/MATLAB Function2' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function1'

  rtb_Gain6 = std::sin(dcm02);
  rtb_integral_t_g_tmp = std::cos(dcm02);
  rtb_accel_n_sp_idx_0 = rtb_integral_t_g_tmp * rtb_accel_yaw_idx_0 + -rtb_Gain6
    * rtb_integral_t_f;
  rtb_accel_n_sp_idx_1 = rtb_Gain6 * rtb_accel_yaw_idx_0 + rtb_integral_t_g_tmp *
    rtb_integral_t_f;

  // MATLAB Function: '<S3>/MATLAB Function1' incorporates:
  //   SignalConversion generated from: '<S65>/ SFunction '

  rtb_accel_yaw_idx_0 = rtb_integral_t_g_tmp * dcm12 + -std::sin(dcm02) *
    rtb_Saturation9;
  rtb_integral_t_g_tmp = rtb_Gain6 * dcm12 + rtb_integral_t_g_tmp *
    rtb_Saturation9;

  // Gain: '<S94>/Gain' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Memory: '<S97>/Memory1'
  //   Sum: '<S94>/Sum5'

  rtb_integral_t_f = (PIDControllerCodeGenrtDW.Memory1_PreviousInput -
                      static_cast<real32_T>(rtu__e_lpe_s_pos_ned[0])) * 2.0F;

  // Gain: '<S94>/Gain1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Memory: '<S96>/Memory1'
  //   Sum: '<S94>/Sum3'

  rtb_Saturation4_idx_1 = (PIDControllerCodeGenrtDW.Memory1_PreviousInput_g -
    static_cast<real32_T>(rtu__e_lpe_s_pos_ned[1])) * 2.0F;

  // Gain: '<S94>/Gain2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Memory: '<S98>/Memory1'
  //   Sum: '<S94>/Sum1'

  rtb_integral_t_l = (PIDControllerCodeGenrtDW.Memory1_PreviousInput_c -
                      static_cast<real32_T>(rtu__e_lpe_s_pos_ned[2])) * 2.0F;

  // MATLAB Function: '<S7>/MATLAB Function3' incorporates:
  //   MATLAB Function: '<S3>/switcher'

  rtb_Gain6_0 = (rtb_DataTypeConversion[7] >= 1500);

  // Saturate: '<S94>/Saturation4'
  if (rtb_integral_t_f > 3.0F) {
    // MATLAB Function: '<S7>/MATLAB Function3'
    rtb_integral_t_f = 3.0F;
  } else if (rtb_integral_t_f < -3.0F) {
    // MATLAB Function: '<S7>/MATLAB Function3'
    rtb_integral_t_f = -3.0F;
  }

  if (rtb_Saturation4_idx_1 > 3.0F) {
    // MATLAB Function: '<S7>/MATLAB Function3'
    rtb_Saturation4_idx_1 = 3.0F;
  } else if (rtb_Saturation4_idx_1 < -3.0F) {
    // MATLAB Function: '<S7>/MATLAB Function3'
    rtb_Saturation4_idx_1 = -3.0F;
  }

  // MATLAB Function: '<S7>/MATLAB Function3' incorporates:
  //   Gain: '<S7>/max vel in vel controller'

  if (!rtb_Gain6_0) {
    rtb_integral_t_f = 3.0F * rtb_accel_yaw_idx_0;
  }

  // Saturate: '<S94>/Saturation4'
  if (rtb_integral_t_l > 3.0F) {
    rtb_integral_t_l = 3.0F;
  } else if (rtb_integral_t_l < -3.0F) {
    rtb_integral_t_l = -3.0F;
  }

  // MATLAB Function: '<S7>/MATLAB Function3' incorporates:
  //   Gain: '<S3>/Gain5'
  //   Gain: '<S7>/max vel in vel controller'

  if (!rtb_Gain6_0) {
    rtb_Saturation4_idx_1 = 3.0F * rtb_integral_t_g_tmp;
    rtb_integral_t_l = 3.0F * -rtb_Gain5;
  }

  // Sum: '<S95>/Sum6' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion1'

  dcm02 = rtb_integral_t_f - static_cast<real32_T>(rtu__e_lpe_s_vel_ned[0]);

  // Sum: '<S95>/Sum4' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion1'

  dcm12 = rtb_Saturation4_idx_1 - static_cast<real32_T>(rtu__e_lpe_s_vel_ned[1]);

  // Sum: '<S95>/Sum2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion1'

  rtb_Saturation9 = rtb_integral_t_l - static_cast<real32_T>
    (rtu__e_lpe_s_vel_ned[2]);

  // Sum: '<S104>/Sum3' incorporates:
  //   Gain: '<S104>/Gain1'
  //   Memory: '<S109>/Memory1'

  rtb_integral_t_f = 5.0F * rtb_Saturation9 +
    PIDControllerCodeGenrtDW.Memory1_PreviousInput_nw;

  // Saturate: '<S95>/Saturation3'
  if (rtb_integral_t_f > 10.0F) {
    rtb_integral_t_f = 10.0F;
  } else if (rtb_integral_t_f < -10.0F) {
    rtb_integral_t_f = -10.0F;
  }

  // Sum: '<S102>/Sum7' incorporates:
  //   Gain: '<S102>/Gain6'
  //   Memory: '<S105>/Memory1'

  rtb_integral_t_l = 2.0F * dcm02 +
    PIDControllerCodeGenrtDW.Memory1_PreviousInput_o;

  // Sum: '<S103>/Sum5' incorporates:
  //   Gain: '<S103>/Gain3'
  //   Memory: '<S107>/Memory1'

  rtb_Rd_idx_0 = 2.0F * dcm12 + PIDControllerCodeGenrtDW.Memory1_PreviousInput_n;

  // MATLAB Function: '<S7>/MATLAB Function2' incorporates:
  //   Saturate: '<S95>/Saturation1'
  //   Saturate: '<S95>/Saturation3'

  rtb_Gain5 = 0.0F;
  rtb_Gain6 = 0.0F;
  if (rtb_y_i == 1.0F) {
    rtb_Gain5 = 1.0F;
    rtb_Gain6 = 1.0F;
  } else if (rtb_y_i == 2.0F) {
    rtb_accel_n_sp_idx_2 = rtb_integral_t_f;
    rtb_Gain6 = 1.0F;
  } else {
    if (rtb_integral_t_l > 6.0F) {
      // Saturate: '<S95>/Saturation1'
      rtb_accel_n_sp_idx_0 = 6.0F;
    } else if (rtb_integral_t_l < -6.0F) {
      // Saturate: '<S95>/Saturation1'
      rtb_accel_n_sp_idx_0 = -6.0F;
    } else {
      rtb_accel_n_sp_idx_0 = rtb_integral_t_l;
    }

    // Saturate: '<S95>/Saturation2' incorporates:
    //   Saturate: '<S95>/Saturation1'

    if (rtb_Rd_idx_0 > 6.0F) {
      rtb_accel_n_sp_idx_1 = 6.0F;
    } else if (rtb_Rd_idx_0 < -6.0F) {
      rtb_accel_n_sp_idx_1 = -6.0F;
    } else {
      rtb_accel_n_sp_idx_1 = rtb_Rd_idx_0;
    }

    // End of Saturate: '<S95>/Saturation2'
    rtb_accel_n_sp_idx_2 = rtb_integral_t_f;
  }

  // End of MATLAB Function: '<S7>/MATLAB Function2'

  // MATLAB Function: '<S3>/deadzone3'
  PIDControllerCodeGen_deadzone1(static_cast<real32_T>(rtb_DataTypeConversion[3]),
    &rtb_integral_t_f);

  // Saturate: '<S3>/Saturation1'
  if (rtb_integral_t_f > 1.0F) {
    rtb_integral_t_f = 1.0F;
  } else if (rtb_integral_t_f < -1.0F) {
    rtb_integral_t_f = -1.0F;
  }

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Gain: '<S3>/Gain9'
  //   Saturate: '<S3>/Saturation1'

  PIDControllerCodeGenrtDW.yaw_angle += 1.04719758F * rtb_integral_t_f * 0.003F;
  rtb_maxvelinvelcontroller[0] = rtb_accel_n_sp_idx_0;
  rtb_maxvelinvelcontroller[1] = rtb_accel_n_sp_idx_1;
  rtb_maxvelinvelcontroller[2] = rtb_accel_n_sp_idx_2 - 9.81F;
  rtb_integral_t_f = norm_afMWvhHe(rtb_maxvelinvelcontroller);
  rtb_y_i = -rtb_accel_n_sp_idx_0 / rtb_integral_t_f;
  rtb_accel_n_sp_idx_0 = -rtb_accel_n_sp_idx_1 / rtb_integral_t_f;
  rtb_integral_t_l = -(rtb_accel_n_sp_idx_2 - 9.81F) / rtb_integral_t_f;
  rtb_integral_t_f = std::cos(PIDControllerCodeGenrtDW.yaw_angle);
  rtb_Saturation4_idx_1 = std::sin(PIDControllerCodeGenrtDW.yaw_angle);
  Yq_idx_2 = rtb_accel_n_sp_idx_0 * 0.0F - rtb_Saturation4_idx_1 *
    rtb_integral_t_l;
  rtb_maxvelinvelcontroller_0[0] = Yq_idx_2;
  rtb_accel_n_sp_idx_1 = rtb_integral_t_f * rtb_integral_t_l - rtb_y_i * 0.0F;
  rtb_maxvelinvelcontroller_0[1] = rtb_accel_n_sp_idx_1;
  rtb_Rd_idx_3 = rtb_y_i * rtb_Saturation4_idx_1 - rtb_integral_t_f *
    rtb_accel_n_sp_idx_0;
  rtb_maxvelinvelcontroller_0[2] = rtb_Rd_idx_3;
  rtb_integral_t_f = norm_afMWvhHe(rtb_maxvelinvelcontroller_0);
  rtb_Saturation4_idx_1 = Yq_idx_2 / rtb_integral_t_f;
  Yq_idx_1 = rtb_accel_n_sp_idx_1 / rtb_integral_t_f;
  Yq_idx_2 = rtb_Rd_idx_3 / rtb_integral_t_f;
  rtb_Rd_idx_6 = Yq_idx_1 * rtb_integral_t_l - rtb_accel_n_sp_idx_0 * Yq_idx_2;
  rtb_maxvelinvelcontroller_0[0] = rtb_Rd_idx_6;
  rtb_Rd_idx_3 = rtb_y_i * Yq_idx_2 - rtb_Saturation4_idx_1 * rtb_integral_t_l;
  rtb_maxvelinvelcontroller_0[1] = rtb_Rd_idx_3;
  rtb_accel_n_sp_idx_1 = rtb_Saturation4_idx_1 * rtb_accel_n_sp_idx_0 - rtb_y_i *
    Yq_idx_1;
  rtb_maxvelinvelcontroller_0[2] = rtb_accel_n_sp_idx_1;
  rtb_integral_t_f = norm_afMWvhHe(rtb_maxvelinvelcontroller_0);
  rtb_Rd_idx_0 = rtb_Rd_idx_6 / rtb_integral_t_f;
  rtb_Rd_idx_3 /= rtb_integral_t_f;
  rtb_Rd_idx_6 = rtb_accel_n_sp_idx_1 / rtb_integral_t_f;

  // Sum: '<S30>/Add' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  rtb_accel_n_sp_idx_1 = (rtb_Rd_idx_0 + Yq_idx_1) + rtb_integral_t_l;

  // If: '<S2>/If' incorporates:
  //   If: '<S27>/Find Maximum Diagonal Value'
  //   MATLAB Function: '<Root>/MATLAB Function1'

  if (rtb_accel_n_sp_idx_1 > 0.0F) {
    // Outputs for IfAction SubSystem: '<S2>/Positive Trace' incorporates:
    //   ActionPort: '<S28>/Action Port'

    // Sqrt: '<S28>/sqrt' incorporates:
    //   Constant: '<S28>/Constant'
    //   Sum: '<S28>/Sum'

    rtb_integral_t_f = std::sqrt(rtb_accel_n_sp_idx_1 + 1.0F);

    // Gain: '<S28>/Gain' incorporates:
    //   Merge: '<S2>/Merge'

    PIDControllerCodeGenrtDW.Merge[0] = 0.5F * rtb_integral_t_f;

    // Gain: '<S28>/Gain1'
    rtb_integral_t_f *= 2.0F;

    // Product: '<S28>/Product' incorporates:
    //   MATLAB Function: '<Root>/MATLAB Function1'
    //   Merge: '<S2>/Merge'
    //   Sum: '<S50>/Add'
    //   Sum: '<S51>/Add'
    //   Sum: '<S52>/Add'

    PIDControllerCodeGenrtDW.Merge[1] = (Yq_idx_2 - rtb_accel_n_sp_idx_0) /
      rtb_integral_t_f;
    PIDControllerCodeGenrtDW.Merge[2] = (rtb_y_i - rtb_Rd_idx_6) /
      rtb_integral_t_f;
    PIDControllerCodeGenrtDW.Merge[3] = (rtb_Rd_idx_3 - rtb_Saturation4_idx_1) /
      rtb_integral_t_f;

    // End of Outputs for SubSystem: '<S2>/Positive Trace'

    // Outputs for IfAction SubSystem: '<S2>/Negative Trace' incorporates:
    //   ActionPort: '<S27>/Action Port'

  } else if ((Yq_idx_1 > rtb_Rd_idx_0) && (Yq_idx_1 > rtb_integral_t_l)) {
    // Outputs for IfAction SubSystem: '<S27>/Maximum Value at DCM(2,2)' incorporates:
    //   ActionPort: '<S32>/Action Port'

    // If: '<S27>/Find Maximum Diagonal Value' incorporates:
    //   Constant: '<S43>/Constant1'
    //   Constant: '<S43>/Constant2'
    //   Constant: '<S44>/Constant'
    //   Gain: '<S32>/Gain'
    //   Gain: '<S32>/Gain1'
    //   Gain: '<S32>/Gain3'
    //   Gain: '<S32>/Gain4'
    //   MATLAB Function: '<Root>/MATLAB Function1'
    //   Merge: '<S2>/Merge'
    //   Product: '<S32>/Product'
    //   Product: '<S43>/Product'
    //   Sqrt: '<S32>/sqrt'
    //   Sum: '<S40>/Add'
    //   Sum: '<S41>/Add'
    //   Sum: '<S42>/Add'
    //   Sum: '<S44>/Add'
    //   Switch: '<S43>/Switch'

    rtb_integral_t_f = std::sqrt(((Yq_idx_1 - rtb_Rd_idx_0) - rtb_integral_t_l)
      + 1.0F);
    if (rtb_integral_t_f != 0.0F) {
      rtb_integral_t_l = 0.5F;
      rtb_accel_n_sp_idx_1 = rtb_integral_t_f;
    } else {
      rtb_integral_t_l = 0.0F;
      rtb_accel_n_sp_idx_1 = 1.0F;
    }

    rtb_integral_t_l /= rtb_accel_n_sp_idx_1;
    PIDControllerCodeGenrtDW.Merge[1] = (rtb_Saturation4_idx_1 + rtb_Rd_idx_3) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[3] = (rtb_accel_n_sp_idx_0 + Yq_idx_2) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[0] = (rtb_y_i - rtb_Rd_idx_6) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[2] = 0.5F * rtb_integral_t_f;

    // End of Outputs for SubSystem: '<S27>/Maximum Value at DCM(2,2)'
  } else if (rtb_integral_t_l > rtb_Rd_idx_0) {
    // Outputs for IfAction SubSystem: '<S27>/Maximum Value at DCM(3,3)' incorporates:
    //   ActionPort: '<S33>/Action Port'

    // If: '<S27>/Find Maximum Diagonal Value' incorporates:
    //   Constant: '<S48>/Constant1'
    //   Constant: '<S48>/Constant2'
    //   Constant: '<S49>/Constant'
    //   Gain: '<S33>/Gain'
    //   Gain: '<S33>/Gain1'
    //   Gain: '<S33>/Gain2'
    //   Gain: '<S33>/Gain3'
    //   MATLAB Function: '<Root>/MATLAB Function1'
    //   Merge: '<S2>/Merge'
    //   Product: '<S33>/Product'
    //   Product: '<S48>/Product'
    //   Sqrt: '<S33>/sqrt'
    //   Sum: '<S45>/Add'
    //   Sum: '<S46>/Add'
    //   Sum: '<S47>/Add'
    //   Sum: '<S49>/Add'
    //   Switch: '<S48>/Switch'

    rtb_integral_t_f = std::sqrt(((rtb_integral_t_l - rtb_Rd_idx_0) - Yq_idx_1)
      + 1.0F);
    if (rtb_integral_t_f != 0.0F) {
      rtb_integral_t_l = 0.5F;
      rtb_accel_n_sp_idx_1 = rtb_integral_t_f;
    } else {
      rtb_integral_t_l = 0.0F;
      rtb_accel_n_sp_idx_1 = 1.0F;
    }

    rtb_integral_t_l /= rtb_accel_n_sp_idx_1;
    PIDControllerCodeGenrtDW.Merge[1] = (rtb_y_i + rtb_Rd_idx_6) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[2] = (rtb_accel_n_sp_idx_0 + Yq_idx_2) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[0] = (rtb_Rd_idx_3 - rtb_Saturation4_idx_1) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[3] = 0.5F * rtb_integral_t_f;

    // End of Outputs for SubSystem: '<S27>/Maximum Value at DCM(3,3)'
  } else {
    // Outputs for IfAction SubSystem: '<S27>/Maximum Value at DCM(1,1)' incorporates:
    //   ActionPort: '<S31>/Action Port'

    // If: '<S27>/Find Maximum Diagonal Value' incorporates:
    //   Constant: '<S38>/Constant1'
    //   Constant: '<S38>/Constant2'
    //   Constant: '<S39>/Constant'
    //   Gain: '<S31>/Gain'
    //   Gain: '<S31>/Gain1'
    //   Gain: '<S31>/Gain2'
    //   Gain: '<S31>/Gain3'
    //   MATLAB Function: '<Root>/MATLAB Function1'
    //   Merge: '<S2>/Merge'
    //   Product: '<S31>/Product'
    //   Product: '<S38>/Product'
    //   Sqrt: '<S31>/sqrt'
    //   Sum: '<S35>/Add'
    //   Sum: '<S36>/Add'
    //   Sum: '<S37>/Add'
    //   Sum: '<S39>/Add'
    //   Switch: '<S38>/Switch'

    rtb_integral_t_f = std::sqrt(((rtb_Rd_idx_0 - Yq_idx_1) - rtb_integral_t_l)
      + 1.0F);
    if (rtb_integral_t_f != 0.0F) {
      rtb_integral_t_l = 0.5F;
      rtb_accel_n_sp_idx_1 = rtb_integral_t_f;
    } else {
      rtb_integral_t_l = 0.0F;
      rtb_accel_n_sp_idx_1 = 1.0F;
    }

    rtb_integral_t_l /= rtb_accel_n_sp_idx_1;
    PIDControllerCodeGenrtDW.Merge[2] = (rtb_Saturation4_idx_1 + rtb_Rd_idx_3) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[3] = (rtb_y_i + rtb_Rd_idx_6) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[0] = (Yq_idx_2 - rtb_accel_n_sp_idx_0) *
      rtb_integral_t_l;
    PIDControllerCodeGenrtDW.Merge[1] = 0.5F * rtb_integral_t_f;

    // End of Outputs for SubSystem: '<S27>/Maximum Value at DCM(1,1)'

    // End of Outputs for SubSystem: '<S2>/Negative Trace'
  }

  // End of If: '<S2>/If'

  // Product: '<S10>/Divide1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   Sum: '<S17>/Sum'
  //   UnaryMinus: '<S16>/Unary Minus'

  rtb_integral_t_f = -static_cast<real32_T>(rtu__e_cf_s_quat_data[1]) /
    rtb_fcn3_k;

  // Product: '<S10>/Divide2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   Sum: '<S17>/Sum'
  //   UnaryMinus: '<S16>/Unary Minus1'

  rtb_accel_n_sp_idx_1 = -static_cast<real32_T>(rtu__e_cf_s_quat_data[2]) /
    rtb_fcn3_k;

  // Product: '<S10>/Divide3' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'
  //   Sum: '<S17>/Sum'
  //   UnaryMinus: '<S16>/Unary Minus2'

  rtb_Rd_idx_0 = -static_cast<real32_T>(rtu__e_cf_s_quat_data[3]) / rtb_fcn3_k;

  // Sum: '<S18>/Sum' incorporates:
  //   Product: '<S18>/Product'
  //   Product: '<S18>/Product1'
  //   Product: '<S18>/Product2'
  //   Product: '<S18>/Product3'

  rtb_y_i = ((rtb_fcn2 * PIDControllerCodeGenrtDW.Merge[0] - rtb_integral_t_f *
              PIDControllerCodeGenrtDW.Merge[1]) - rtb_accel_n_sp_idx_1 *
             PIDControllerCodeGenrtDW.Merge[2]) - rtb_Rd_idx_0 *
    PIDControllerCodeGenrtDW.Merge[3];

  // Sum: '<S19>/Sum' incorporates:
  //   Product: '<S19>/Product'
  //   Product: '<S19>/Product1'
  //   Product: '<S19>/Product2'
  //   Product: '<S19>/Product3'

  rtb_Saturation4_idx_1 = ((rtb_fcn2 * PIDControllerCodeGenrtDW.Merge[1] +
    rtb_integral_t_f * PIDControllerCodeGenrtDW.Merge[0]) + rtb_accel_n_sp_idx_1
    * PIDControllerCodeGenrtDW.Merge[3]) - rtb_Rd_idx_0 *
    PIDControllerCodeGenrtDW.Merge[2];

  // Sum: '<S20>/Sum' incorporates:
  //   Product: '<S20>/Product'
  //   Product: '<S20>/Product1'
  //   Product: '<S20>/Product2'
  //   Product: '<S20>/Product3'

  rtb_integral_t_l = ((rtb_fcn2 * PIDControllerCodeGenrtDW.Merge[2] -
                       rtb_integral_t_f * PIDControllerCodeGenrtDW.Merge[3]) +
                      rtb_accel_n_sp_idx_1 * PIDControllerCodeGenrtDW.Merge[0])
    + rtb_Rd_idx_0 * PIDControllerCodeGenrtDW.Merge[1];

  // Sum: '<S21>/Sum' incorporates:
  //   Product: '<S21>/Product'
  //   Product: '<S21>/Product1'
  //   Product: '<S21>/Product2'
  //   Product: '<S21>/Product3'

  rtb_fcn2 = ((rtb_fcn2 * PIDControllerCodeGenrtDW.Merge[3] + rtb_integral_t_f *
               PIDControllerCodeGenrtDW.Merge[2]) - rtb_accel_n_sp_idx_1 *
              PIDControllerCodeGenrtDW.Merge[1]) + rtb_Rd_idx_0 *
    PIDControllerCodeGenrtDW.Merge[0];

  // Sqrt: '<S22>/sqrt' incorporates:
  //   Product: '<S23>/Product'
  //   Product: '<S23>/Product1'
  //   Product: '<S23>/Product2'
  //   Product: '<S23>/Product3'
  //   Sum: '<S23>/Sum'

  rtb_integral_t_f = std::sqrt(((rtb_y_i * rtb_y_i + rtb_Saturation4_idx_1 *
    rtb_Saturation4_idx_1) + rtb_integral_t_l * rtb_integral_t_l) + rtb_fcn2 *
    rtb_fcn2);

  // SignalConversion generated from: '<S9>/ SFunction ' incorporates:
  //   MATLAB Function: '<S1>/MATLAB Function3'
  //   Product: '<S12>/Product'
  //   Product: '<S12>/Product1'
  //   Product: '<S12>/Product2'
  //   Product: '<S12>/Product3'
  //   SignalConversion generated from: '<Root>/Bus Creator'

  rtb_TmpSignalConversionAtBusC_a[0] = rtb_y_i / rtb_integral_t_f;
  rtb_TmpSignalConversionAtBusC_a[1] = rtb_Saturation4_idx_1 / rtb_integral_t_f;
  rtb_TmpSignalConversionAtBusC_a[2] = rtb_integral_t_l / rtb_integral_t_f;
  rtb_TmpSignalConversionAtBusC_a[3] = rtb_fcn2 / rtb_integral_t_f;

  // MATLAB Function: '<S1>/MATLAB Function3' incorporates:
  //   SignalConversion generated from: '<Root>/Bus Creator'

  rtb_integral_t_f = 2.0F * std::acos(std::abs(rtb_TmpSignalConversionAtBusC_a[0]));
  if (rtb_integral_t_f == 0.0F) {
    rtb_maxvelinvelcontroller[0] = 0.0F;
    rtb_maxvelinvelcontroller[1] = 0.0F;
    rtb_maxvelinvelcontroller[2] = 0.0F;
  } else {
    if (std::isnan(rtb_TmpSignalConversionAtBusC_a[0])) {
      rtb_fcn2 = (rtNaNF);
    } else if (rtb_TmpSignalConversionAtBusC_a[0] < 0.0F) {
      rtb_fcn2 = -1.0F;
    } else {
      rtb_fcn2 = (rtb_TmpSignalConversionAtBusC_a[0] > 0.0F);
    }

    rtb_fcn2 = rtb_fcn2 * rtb_integral_t_f / 2.0F / std::sin(rtb_integral_t_f /
      2.0F);
    rtb_maxvelinvelcontroller[0] = rtb_fcn2 * rtb_TmpSignalConversionAtBusC_a[1];
    rtb_maxvelinvelcontroller[2] = rtb_fcn2 * rtb_TmpSignalConversionAtBusC_a[3];
    rtb_maxvelinvelcontroller[1] = rtb_fcn2 * rtb_TmpSignalConversionAtBusC_a[2]
      * 0.8F;
  }

  // Gain: '<S1>/Gain6'
  rtb_y_i = 5.0F * rtb_maxvelinvelcontroller[0];
  rtb_accel_n_sp_idx_0 = 5.0F * rtb_maxvelinvelcontroller[1];
  rtb_integral_t_l = 5.0F * rtb_maxvelinvelcontroller[2];

  // Saturate: '<S1>/Saturation4'
  if (rtb_integral_t_l > 1.57079637F) {
    rtb_integral_t_l = 1.57079637F;
  } else if (rtb_integral_t_l < -1.57079637F) {
    rtb_integral_t_l = -1.57079637F;
  }

  // Sum: '<S1>/Sum1' incorporates:
  //   Saturate: '<S1>/Saturation4'

  rtb_integral_t_l -= rtu__m_gyro_s_gyro_data[2];

  // Saturate: '<S1>/Saturation5'
  if (rtb_y_i > 3.14159274F) {
    rtb_y_i = 3.14159274F;
  } else if (rtb_y_i < -3.14159274F) {
    rtb_y_i = -3.14159274F;
  }

  // Sum: '<S1>/Sum21' incorporates:
  //   Saturate: '<S1>/Saturation5'

  rtb_Rd_idx_0 = rtb_y_i - rtu__m_gyro_s_gyro_data[0];

  // Saturate: '<S1>/Saturation3'
  if (rtb_accel_n_sp_idx_0 > 3.14159274F) {
    rtb_accel_n_sp_idx_0 = 3.14159274F;
  } else if (rtb_accel_n_sp_idx_0 < -3.14159274F) {
    rtb_accel_n_sp_idx_0 = -3.14159274F;
  }

  // Sum: '<S1>/Sum22' incorporates:
  //   Saturate: '<S1>/Saturation3'

  rtb_integral_t_f = rtb_accel_n_sp_idx_0 - rtu__m_gyro_s_gyro_data[1];

  // SampleTimeMath: '<S25>/TSamp' incorporates:
  //   Gain: '<S14>/Derivative Gain'
  //
  //  About '<S25>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_fcn2 = 0.0F * rtb_Rd_idx_0 * 1000.0F;

  // SampleTimeMath: '<S24>/TSamp' incorporates:
  //   Gain: '<S13>/Derivative Gain'
  //
  //  About '<S24>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Saturation4_idx_1 = 0.0F * rtb_integral_t_f * 1000.0F;

  // SampleTimeMath: '<S26>/TSamp' incorporates:
  //   Gain: '<S15>/Derivative Gain'
  //
  //  About '<S26>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  Yq_idx_2 = 0.0F * rtb_integral_t_l * 1000.0F;

  // Gain: '<S13>/Integral Gain'
  Yq_idx_1 = 0.0F * rtb_integral_t_f;

  // Gain: '<S14>/Integral Gain'
  rtb_Rd_idx_3 = 0.0F * rtb_Rd_idx_0;

  // Gain: '<S15>/Integral Gain'
  rtb_Rd_idx_6 = 0.0F * rtb_integral_t_l;

  // MATLAB Function: '<S8>/pwm_out1' incorporates:
  //   Constant: '<Root>/Constant2'
  //   Constant: '<S8>/Constant'
  //   Constant: '<S8>/Constant1'
  //   Constant: '<S8>/Constant2'
  //   Gain: '<S13>/Gain'
  //   Gain: '<S14>/Gain'
  //   Gain: '<S15>/Gain'
  //   Product: '<S8>/Divide'
  //   Sum: '<S13>/Sum'
  //   Sum: '<S14>/Sum'
  //   Sum: '<S15>/Sum'
  //   Sum: '<S24>/Diff'
  //   Sum: '<S25>/Diff'
  //   Sum: '<S26>/Diff'
  //   Sum: '<S8>/Sum1'
  //   UnitDelay: '<S24>/UD'
  //   UnitDelay: '<S25>/UD'
  //   UnitDelay: '<S26>/UD'
  //
  //  Block description for '<S24>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S25>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S26>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S24>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S25>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S26>/UD':
  //
  //   Store in Global RAM

  tmp[0] = 0.0F;
  tmp[4] = 0.0F;
  tmp[8] = -7.14285707F;
  tmp[12] = -4.0F;
  tmp[1] = -0.5F;
  tmp[5] = -4.0F;
  tmp[9] = 0.0F;
  tmp[13] = 0.0F;
  tmp[2] = 0.0F;
  tmp[6] = 0.0F;
  tmp[10] = -7.14285707F;
  tmp[14] = 4.0F;
  tmp[3] = -0.5F;
  tmp[7] = 4.0F;
  tmp[11] = 0.0F;
  tmp[15] = 0.0F;
  rtb_accel_n_sp_idx_0 = static_cast<real32_T>((rtb_accel_n_sp_idx_2 - 9.81F) *
    0.742);
  rtb_accel_n_sp_idx_1 = 0.1F * rtb_Rd_idx_0 + (rtb_fcn2 -
    PIDControllerCodeGenrtDW.UD_DSTATE);
  rtb_accel_n_sp_idx_2 = 0.05F * rtb_integral_t_f + (rtb_Saturation4_idx_1 -
    PIDControllerCodeGenrtDW.UD_DSTATE_n);
  rtb_integral_t_f = 0.1F * rtb_integral_t_l + (Yq_idx_2 -
    PIDControllerCodeGenrtDW.UD_DSTATE_nq);
  for (int32_T i{0}; i < 4; i++) {
    rtb_TmpSignalConversionAtBusC_a[i] = ((tmp[i + 4] * rtb_accel_n_sp_idx_1 +
      tmp[i] * rtb_accel_n_sp_idx_0) + tmp[i + 8] * rtb_accel_n_sp_idx_2) +
      tmp[i + 12] * rtb_integral_t_f;
  }

  rtb_integral_t_l = std::sqrt(rtb_TmpSignalConversionAtBusC_a[0] *
    rtb_TmpSignalConversionAtBusC_a[0] + rtb_TmpSignalConversionAtBusC_a[1] *
    rtb_TmpSignalConversionAtBusC_a[1]);
  rtb_Rd_idx_0 = std::sqrt(rtb_TmpSignalConversionAtBusC_a[2] *
    rtb_TmpSignalConversionAtBusC_a[2] + rtb_TmpSignalConversionAtBusC_a[3] *
    rtb_TmpSignalConversionAtBusC_a[3]);
  rtb_accel_n_sp_idx_2 = rt_atan2f_snf(rtb_TmpSignalConversionAtBusC_a[0],
    rtb_TmpSignalConversionAtBusC_a[1]);
  rtb_integral_t_f = rt_atan2f_snf(rtb_TmpSignalConversionAtBusC_a[2],
    rtb_TmpSignalConversionAtBusC_a[3]);

  // End of MATLAB Function: '<S8>/pwm_out1'

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   BusCreator: '<Root>/Bus Creator'

  *rty__c_out_s_time_stamp = *rtu_usec;

  // SignalConversion generated from: '<Root>/_s_scope_s' incorporates:
  //   BusCreator: '<Root>/Bus Creator2'

  *rty__s_scope_s_time_stamp = *rtu__m_gyro_s_time_stamp;

  // Saturate: '<S8>/Saturation'
  if (rtb_integral_t_l > 14.0F) {
    rtb_integral_t_l = 14.0F;
  }

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion3'
  //   DataTypeConversion: '<S8>/Data Type Conversion2'
  //   MATLAB Function: '<S8>/MATLAB Function'
  //   Saturate: '<S8>/Saturation'

  rty__c_out_s_pwm[0] = static_cast<uint16_T>(std::sqrt(rtb_integral_t_l / 14.0)
    * 1000.0 + 1000.0);

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   SignalConversion generated from: '<Root>/Bus Creator'

  rty__c_out_s_thrust[0] = 0.0F;

  // Saturate: '<S8>/Saturation'
  if (rtb_Rd_idx_0 > 14.0F) {
    rtb_Rd_idx_0 = 14.0F;
  }

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion3'
  //   DataTypeConversion: '<S8>/Data Type Conversion2'
  //   MATLAB Function: '<S8>/MATLAB Function'
  //   Saturate: '<S8>/Saturation'

  rty__c_out_s_pwm[1] = static_cast<uint16_T>(std::sqrt(rtb_Rd_idx_0 / 14.0) *
    1000.0 + 1000.0);

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   SignalConversion generated from: '<Root>/Bus Creator'

  rty__c_out_s_thrust[1] = 0.0F;

  // Saturate: '<S8>/Saturation1'
  if (rtb_accel_n_sp_idx_2 > 0.785398185F) {
    rtb_accel_n_sp_idx_2 = 0.785398185F;
  } else if (rtb_accel_n_sp_idx_2 < -0.785398185F) {
    rtb_accel_n_sp_idx_2 = -0.785398185F;
  }

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion3'
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   MATLAB Function: '<S8>/MATLAB Function'
  //   Saturate: '<S8>/Saturation1'

  rty__c_out_s_pwm[2] = static_cast<uint16_T>(rtb_accel_n_sp_idx_2 /
    1.5707963705062866 * 500.0 + 1500.0);

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   SignalConversion generated from: '<Root>/Bus Creator'

  rty__c_out_s_thrust[2] = 0.0F;

  // Saturate: '<S8>/Saturation1'
  if (rtb_integral_t_f > 0.785398185F) {
    rtb_integral_t_f = 0.785398185F;
  } else if (rtb_integral_t_f < -0.785398185F) {
    rtb_integral_t_f = -0.785398185F;
  }

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion3'
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   MATLAB Function: '<S8>/MATLAB Function'
  //   Saturate: '<S8>/Saturation1'

  rty__c_out_s_pwm[3] = static_cast<uint16_T>(rtb_integral_t_f /
    1.5707963705062866 * 500.0 + 1500.0);

  // SignalConversion generated from: '<Root>/_c_out_s' incorporates:
  //   SignalConversion generated from: '<Root>/Bus Creator'

  rty__c_out_s_thrust[3] = 0.0F;

  // SignalConversion generated from: '<Root>/_s_scope_s' incorporates:
  //   BusCreator: '<Root>/Bus Creator2'
  //   Constant: '<Root>/Constant4'

  *rty__s_scope_s_rate_hz = 50U;

  // SignalConversion generated from: '<Root>/_s_scope_s' incorporates:
  //   SignalConversion generated from: '<Root>/Bus Creator2'

  rty__s_scope_s_data[0] = 0.0F;
  rty__s_scope_s_data[1] = 0.0F;
  rty__s_scope_s_data[3] = 0.0F;
  rty__s_scope_s_data[2] = 0.0F;
  rty__s_scope_s_data[4] = 0.0F;
  for (int32_T i{0}; i < 35; i++) {
    rty__s_scope_s_data[i + 5] = 0.0F;
  }

  // Sqrt: '<S90>/sqrt'
  rtb_integral_t_f = std::sqrt(rtb_fcn3_k);

  // Product: '<S85>/Product' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'

  rtb_integral_t_l = static_cast<real32_T>(rtu__e_cf_s_quat_data[0]) /
    rtb_integral_t_f;

  // Product: '<S85>/Product1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'

  rtb_Rd_idx_0 = static_cast<real32_T>(rtu__e_cf_s_quat_data[1]) /
    rtb_integral_t_f;

  // Product: '<S85>/Product2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'

  rtb_accel_n_sp_idx_1 = static_cast<real32_T>(rtu__e_cf_s_quat_data[2]) /
    rtb_integral_t_f;

  // Product: '<S85>/Product3' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion13'

  rtb_integral_t_f = static_cast<real32_T>(rtu__e_cf_s_quat_data[3]) /
    rtb_integral_t_f;

  // Fcn: '<S6>/fcn3'
  rtb_fcn3_k = (rtb_Rd_idx_0 * rtb_integral_t_f - rtb_integral_t_l *
                rtb_accel_n_sp_idx_1) * -2.0F;

  // If: '<S86>/If' incorporates:
  //   Constant: '<S87>/Constant'
  //   Constant: '<S88>/Constant'

  if (rtb_fcn3_k > 1.0F) {
    // Outputs for IfAction SubSystem: '<S86>/If Action Subsystem' incorporates:
    //   ActionPort: '<S87>/Action Port'

    rtb_y_i = 1.0F;

    // End of Outputs for SubSystem: '<S86>/If Action Subsystem'
  } else if (rtb_fcn3_k < -1.0F) {
    // Outputs for IfAction SubSystem: '<S86>/If Action Subsystem1' incorporates:
    //   ActionPort: '<S88>/Action Port'

    rtb_y_i = 1.0F;

    // End of Outputs for SubSystem: '<S86>/If Action Subsystem1'
  } else {
    // Outputs for IfAction SubSystem: '<S86>/If Action Subsystem2' incorporates:
    //   ActionPort: '<S89>/Action Port'

    // SignalConversion generated from: '<S89>/In'
    rtb_y_i = rtb_fcn3_k;

    // End of Outputs for SubSystem: '<S86>/If Action Subsystem2'
  }

  // End of If: '<S86>/If'

  // Fcn: '<S6>/fcn4'
  rtb_fcn3_k = (rtb_accel_n_sp_idx_1 * rtb_integral_t_f + rtb_integral_t_l *
                rtb_Rd_idx_0) * 2.0F;

  // Fcn: '<S6>/fcn5'
  rtb_integral_t_l = ((rtb_integral_t_l * rtb_integral_t_l - rtb_Rd_idx_0 *
                       rtb_Rd_idx_0) - rtb_accel_n_sp_idx_1 *
                      rtb_accel_n_sp_idx_1) + rtb_integral_t_f *
    rtb_integral_t_f;

  // MATLAB Function: '<S96>/MATLAB Function2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Memory: '<S96>/Memory1'

  PIDControllerCo_MATLABFunction2
    (PIDControllerCodeGenrtDW.Memory1_PreviousInput_g, 0.003F,
     rtb_integral_t_g_tmp, rtb_Gain6, static_cast<real32_T>
     (rtu__e_lpe_s_pos_ned[1]), &rtb_fcn3_k);

  // MATLAB Function: '<S97>/MATLAB Function2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Memory: '<S97>/Memory1'

  PIDControllerCo_MATLABFunction2(PIDControllerCodeGenrtDW.Memory1_PreviousInput,
    0.003F, rtb_accel_yaw_idx_0, rtb_Gain6, static_cast<real32_T>
    (rtu__e_lpe_s_pos_ned[0]), &rtb_y_i);

  // MATLAB Function: '<S98>/MATLAB Function2' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Memory: '<S98>/Memory1'

  PIDControllerCo_MATLABFunction2
    (PIDControllerCodeGenrtDW.Memory1_PreviousInput_c, 0.003F,
     rtb_accel_yaw_idx_2, rtb_Gain5, static_cast<real32_T>(rtu__e_lpe_s_pos_ned
      [2]), &rtb_accel_n_sp_idx_1);

  // MATLAB Function: '<S105>/MATLAB Function2' incorporates:
  //   Constant: '<S102>/Constant4'
  //   Gain: '<S102>/Gain5'
  //   Memory: '<S105>/Memory1'

  PIDController_MATLABFunction2_i
    (PIDControllerCodeGenrtDW.Memory1_PreviousInput_o, 0.003F, 0.1F * dcm02,
     3.0F, rtb_Gain6, &rtb_Rd_idx_0);

  // MATLAB Function: '<S107>/MATLAB Function2' incorporates:
  //   Constant: '<S103>/Constant3'
  //   Gain: '<S103>/Gain2'
  //   Memory: '<S107>/Memory1'

  PIDController_MATLABFunction2_i
    (PIDControllerCodeGenrtDW.Memory1_PreviousInput_n, 0.003F, 0.1F * dcm12,
     3.0F, rtb_Gain6, &rtb_integral_t_l);

  // MATLAB Function: '<S109>/MATLAB Function2' incorporates:
  //   Constant: '<S104>/Constant2'
  //   Gain: '<S104>/Gain'
  //   Memory: '<S109>/Memory1'

  PIDController_MATLABFunction2_i
    (PIDControllerCodeGenrtDW.Memory1_PreviousInput_nw, 0.003F, 0.1F *
     rtb_Saturation9, 5.0F, rtb_Gain5, &rtb_integral_t_f);

  // Update for Memory: '<S97>/Memory1'
  PIDControllerCodeGenrtDW.Memory1_PreviousInput = rtb_y_i;

  // Update for Memory: '<S96>/Memory1'
  PIDControllerCodeGenrtDW.Memory1_PreviousInput_g = rtb_fcn3_k;

  // Update for Memory: '<S98>/Memory1'
  PIDControllerCodeGenrtDW.Memory1_PreviousInput_c = rtb_accel_n_sp_idx_1;

  // Update for Memory: '<S105>/Memory1'
  PIDControllerCodeGenrtDW.Memory1_PreviousInput_o = rtb_Rd_idx_0;

  // Update for Memory: '<S107>/Memory1'
  PIDControllerCodeGenrtDW.Memory1_PreviousInput_n = rtb_integral_t_l;

  // Update for Memory: '<S109>/Memory1'
  PIDControllerCodeGenrtDW.Memory1_PreviousInput_nw = rtb_integral_t_f;

  // Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
  PIDControllerCodeGenrtDW.DiscreteTimeIntegrator_DSTATE = 0.001F * rtb_Rd_idx_3;
  PIDControllerCodeGenrtDW.DiscreteTimeIntegrator_PrevRese = 1;

  // Update for UnitDelay: '<S25>/UD'
  //
  //  Block description for '<S25>/UD':
  //
  //   Store in Global RAM

  PIDControllerCodeGenrtDW.UD_DSTATE = rtb_fcn2;

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  PIDControllerCodeGenrtDW.DiscreteTimeIntegrator_DSTATE_n = 0.001F * Yq_idx_1;
  PIDControllerCodeGenrtDW.DiscreteTimeIntegrator_PrevRe_i = 1;

  // Update for UnitDelay: '<S24>/UD'
  //
  //  Block description for '<S24>/UD':
  //
  //   Store in Global RAM

  PIDControllerCodeGenrtDW.UD_DSTATE_n = rtb_Saturation4_idx_1;

  // Update for DiscreteIntegrator: '<S15>/Discrete-Time Integrator'
  PIDControllerCodeGenrtDW.DiscreteTimeIntegrator_DSTATE_d = 0.001F *
    rtb_Rd_idx_6;
  PIDControllerCodeGenrtDW.DiscreteTimeIntegrator_PrevRe_b = 1;

  // Update for UnitDelay: '<S26>/UD'
  //
  //  Block description for '<S26>/UD':
  //
  //   Store in Global RAM

  PIDControllerCodeGenrtDW.UD_DSTATE_nq = Yq_idx_2;
}

// Model initialize function
void PIDControllerCodeGen::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));
}

// Constructor
PIDControllerCodeGen::PIDControllerCodeGen() :
  PIDControllerCodeGenrtDW(),
  PIDControllerCodeGenrtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
PIDControllerCodeGen::~PIDControllerCodeGen()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
PIDControllerCodeGen::PIDControllerCodeGen_RT_MODEL * PIDControllerCodeGen::
  getRTM()
{
  return (&PIDControllerCodeGenrtM);
}

// member function to setup error status pointer
void PIDControllerCodeGen::setErrorStatusPointer(const char_T **rt_errorStatus)
{
  rtmSetErrorStatusPointer((&PIDControllerCodeGenrtM), rt_errorStatus);
}

//
// File trailer for generated code.
//
// [EOF]
//
