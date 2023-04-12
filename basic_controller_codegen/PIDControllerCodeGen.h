//
// File: PIDControllerCodeGen.h
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
#ifndef RTW_HEADER_PIDControllerCodeGen_h_
#define RTW_HEADER_PIDControllerCodeGen_h_
#include "rtwtypes.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

#include <cstring>
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

// Class declaration for model PIDControllerCodeGen
class PIDControllerCodeGen final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for model 'PIDControllerCodeGen' 
  struct PIDControllerCodeGen_DW {
    real32_T Merge[4];                 // '<S2>/Merge'
    real32_T DiscreteTimeIntegrator_DSTATE;// '<S14>/Discrete-Time Integrator'
    real32_T UD_DSTATE;                // '<S25>/UD'
    real32_T DiscreteTimeIntegrator_DSTATE_n;// '<S13>/Discrete-Time Integrator' 
    real32_T UD_DSTATE_n;              // '<S24>/UD'
    real32_T DiscreteTimeIntegrator_DSTATE_d;// '<S15>/Discrete-Time Integrator' 
    real32_T UD_DSTATE_nq;             // '<S26>/UD'
    real32_T Memory1_PreviousInput;    // '<S97>/Memory1'
    real32_T Memory1_PreviousInput_g;  // '<S96>/Memory1'
    real32_T Memory1_PreviousInput_c;  // '<S98>/Memory1'
    real32_T Memory1_PreviousInput_o;  // '<S105>/Memory1'
    real32_T Memory1_PreviousInput_n;  // '<S107>/Memory1'
    real32_T Memory1_PreviousInput_nw; // '<S109>/Memory1'
    real32_T yaw_angle;                // '<Root>/MATLAB Function1'
    int8_T DiscreteTimeIntegrator_PrevRese;// '<S14>/Discrete-Time Integrator'
    int8_T DiscreteTimeIntegrator_PrevRe_i;// '<S13>/Discrete-Time Integrator'
    int8_T DiscreteTimeIntegrator_PrevRe_b;// '<S15>/Discrete-Time Integrator'
  };

  // Real-time Model Data Structure
  struct PIDControllerCodeGen_RT_MODEL {
    const char_T **errorStatus;
  };

  // model initialize function
  void initialize();

  // Initial conditions function
  void init();

  // Copy Constructor
  PIDControllerCodeGen(PIDControllerCodeGen const&) = delete;

  // Assignment Operator
  PIDControllerCodeGen& operator= (PIDControllerCodeGen const&) & = delete;

  // Move Constructor
  PIDControllerCodeGen(PIDControllerCodeGen &&) = delete;

  // Move Assignment Operator
  PIDControllerCodeGen& operator= (PIDControllerCodeGen &&) = delete;

  // Real-Time Model get method
  PIDControllerCodeGen::PIDControllerCodeGen_RT_MODEL * getRTM();

  //member function to setup error status pointer
  void setErrorStatusPointer(const char_T **rt_errorStatus);

  // model step function
  void step(const uint64_T *rtu_usec, const real_T rtu__e_lpe_s_pos_ned[3],
            const real_T rtu__e_lpe_s_vel_ned[3], const real_T
            rtu__e_cf_s_quat_data[4], const uint16_T rtu__c_subs_s_channels[16],
            const uint64_T *rtu__m_gyro_s_time_stamp, const real32_T
            rtu__m_gyro_s_gyro_data[3], uint64_T *rty__c_out_s_time_stamp,
            uint16_T rty__c_out_s_pwm[4], real32_T rty__c_out_s_thrust[4],
            uint64_T *rty__s_scope_s_time_stamp, uint32_T
            *rty__s_scope_s_rate_hz, real32_T rty__s_scope_s_data[40]);

  // Constructor
  PIDControllerCodeGen();

  // Destructor
  ~PIDControllerCodeGen();

  // private data and function members
 private:
  // Block states
  PIDControllerCodeGen_DW PIDControllerCodeGenrtDW;

  // private member function(s) for subsystem '<S3>/deadzone1'
  static void PIDControllerCodeGen_deadzone1(real32_T rtu_u, real32_T *rty_y);

  // private member function(s) for subsystem '<S96>/MATLAB Function2'
  static void PIDControllerCo_MATLABFunction2(real32_T rtu_integral_0, real32_T
    rtu_deltaT, real32_T rtu_u, real32_T rtu_reset, real32_T rtu_reset_value,
    real32_T *rty_integral_t);

  // private member function(s) for subsystem '<S105>/MATLAB Function2'
  static void PIDController_MATLABFunction2_i(real32_T rtu_integral_0, real32_T
    rtu_deltaT, real32_T rtu_u, real32_T rtu_max_value, real32_T rtu_reset,
    real32_T *rty_integral_t);

  // Real-Time Model
  PIDControllerCodeGen_RT_MODEL PIDControllerCodeGenrtM;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S24>/Data Type Duplicate' : Unused code path elimination
//  Block '<S25>/Data Type Duplicate' : Unused code path elimination
//  Block '<S26>/Data Type Duplicate' : Unused code path elimination
//  Block '<Root>/Gain' : Unused code path elimination
//  Block '<Root>/Gain1' : Unused code path elimination
//  Block '<Root>/Gain2' : Unused code path elimination
//  Block '<S3>/Display' : Unused code path elimination
//  Block '<S3>/Gain' : Unused code path elimination
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Sum1' : Unused code path elimination
//  Block '<Root>/Data Type Conversion14' : Eliminate redundant data type conversion
//  Block '<Root>/Data Type Conversion15' : Eliminate redundant data type conversion
//  Block '<S2>/Reshape 3x3 -> 9' : Reshape block reduction
//  Block '<S56>/Reshape' : Reshape block reduction
//  Block '<S63>/Reshape' : Reshape block reduction
//  Block '<S3>/Gain12' : Eliminated nontunable gain of 1
//  Block '<S3>/Gain13' : Eliminated nontunable gain of 1
//  Block '<S3>/Gain2' : Eliminated nontunable gain of 1
//  Block '<S3>/Gain8' : Eliminated nontunable gain of 1
//  Block '<S7>/max vel in pos control' : Eliminated nontunable gain of 1
//  Block '<S1>/Constant1' : Unused code path elimination
//  Block '<S1>/full_attitude' : Unused code path elimination
//  Block '<S3>/Gain1' : Unused code path elimination
//  Block '<S3>/Gain3' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PIDControllerCodeGen'
//  '<S1>'   : 'PIDControllerCodeGen/AttitudeControl'
//  '<S2>'   : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions'
//  '<S3>'   : 'PIDControllerCodeGen/InputConditioning1'
//  '<S4>'   : 'PIDControllerCodeGen/MATLAB Function1'
//  '<S5>'   : 'PIDControllerCodeGen/Quaternions to Rotation Angles'
//  '<S6>'   : 'PIDControllerCodeGen/Quaternions to Rotation Angles1'
//  '<S7>'   : 'PIDControllerCodeGen/Subsystem1'
//  '<S8>'   : 'PIDControllerCodeGen/Subsystem3'
//  '<S9>'   : 'PIDControllerCodeGen/AttitudeControl/MATLAB Function3'
//  '<S10>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Inverse'
//  '<S11>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Multiplication'
//  '<S12>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Normalize'
//  '<S13>'  : 'PIDControllerCodeGen/AttitudeControl/pitch_rate'
//  '<S14>'  : 'PIDControllerCodeGen/AttitudeControl/roll_rate'
//  '<S15>'  : 'PIDControllerCodeGen/AttitudeControl/yaw_rate'
//  '<S16>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Inverse/Quaternion Conjugate'
//  '<S17>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Inverse/Quaternion Norm'
//  '<S18>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Multiplication/q0'
//  '<S19>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Multiplication/q1'
//  '<S20>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Multiplication/q2'
//  '<S21>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Multiplication/q3'
//  '<S22>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Normalize/Quaternion Modulus'
//  '<S23>'  : 'PIDControllerCodeGen/AttitudeControl/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S24>'  : 'PIDControllerCodeGen/AttitudeControl/pitch_rate/Discrete Derivative'
//  '<S25>'  : 'PIDControllerCodeGen/AttitudeControl/roll_rate/Discrete Derivative'
//  '<S26>'  : 'PIDControllerCodeGen/AttitudeControl/yaw_rate/Discrete Derivative'
//  '<S27>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace'
//  '<S28>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Positive Trace'
//  '<S29>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM'
//  '<S30>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/trace(DCM)'
//  '<S31>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
//  '<S32>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
//  '<S33>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
//  '<S34>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
//  '<S35>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S36>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S37>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S38>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
//  '<S39>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
//  '<S40>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S41>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S42>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S43>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
//  '<S44>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
//  '<S45>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
//  '<S46>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S47>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S48>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
//  '<S49>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
//  '<S50>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
//  '<S51>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
//  '<S52>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
//  '<S53>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
//  '<S54>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
//  '<S55>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
//  '<S56>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
//  '<S57>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
//  '<S58>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
//  '<S59>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
//  '<S60>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
//  '<S61>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
//  '<S62>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
//  '<S63>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
//  '<S64>'  : 'PIDControllerCodeGen/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
//  '<S65>'  : 'PIDControllerCodeGen/InputConditioning1/MATLAB Function1'
//  '<S66>'  : 'PIDControllerCodeGen/InputConditioning1/MATLAB Function2'
//  '<S67>'  : 'PIDControllerCodeGen/InputConditioning1/MATLAB Function3'
//  '<S68>'  : 'PIDControllerCodeGen/InputConditioning1/arm_switch'
//  '<S69>'  : 'PIDControllerCodeGen/InputConditioning1/deadzone1'
//  '<S70>'  : 'PIDControllerCodeGen/InputConditioning1/deadzone2'
//  '<S71>'  : 'PIDControllerCodeGen/InputConditioning1/deadzone3'
//  '<S72>'  : 'PIDControllerCodeGen/InputConditioning1/deadzone4'
//  '<S73>'  : 'PIDControllerCodeGen/InputConditioning1/flight_mode'
//  '<S74>'  : 'PIDControllerCodeGen/InputConditioning1/quat2eul'
//  '<S75>'  : 'PIDControllerCodeGen/InputConditioning1/switcher'
//  '<S76>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Angle Calculation'
//  '<S77>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Quaternion Normalize'
//  '<S78>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input'
//  '<S79>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S80>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S81>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S82>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus'
//  '<S83>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S84>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Angle Calculation'
//  '<S85>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Quaternion Normalize'
//  '<S86>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input'
//  '<S87>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem'
//  '<S88>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem1'
//  '<S89>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Angle Calculation/Protect asincos input/If Action Subsystem2'
//  '<S90>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Quaternion Normalize/Quaternion Modulus'
//  '<S91>'  : 'PIDControllerCodeGen/Quaternions to Rotation Angles1/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S92>'  : 'PIDControllerCodeGen/Subsystem1/MATLAB Function2'
//  '<S93>'  : 'PIDControllerCodeGen/Subsystem1/MATLAB Function3'
//  '<S94>'  : 'PIDControllerCodeGen/Subsystem1/Subsystem'
//  '<S95>'  : 'PIDControllerCodeGen/Subsystem1/Subsystem4'
//  '<S96>'  : 'PIDControllerCodeGen/Subsystem1/Subsystem/Integral1'
//  '<S97>'  : 'PIDControllerCodeGen/Subsystem1/Subsystem/Integral2'
//  '<S98>'  : 'PIDControllerCodeGen/Subsystem1/Subsystem/Integral3'
//  '<S99>'  : 'PIDControllerCodeGen/Subsystem1/Subsystem/Integral1/MATLAB Function2'
//  '<S100>' : 'PIDControllerCodeGen/Subsystem1/Subsystem/Integral2/MATLAB Function2'
//  '<S101>' : 'PIDControllerCodeGen/Subsystem1/Subsystem/Integral3/MATLAB Function2'
//  '<S102>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem'
//  '<S103>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem1'
//  '<S104>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem2'
//  '<S105>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem/Integral2'
//  '<S106>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem/Integral2/MATLAB Function2'
//  '<S107>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem1/Integral1'
//  '<S108>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem1/Integral1/MATLAB Function2'
//  '<S109>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem2/Integral'
//  '<S110>' : 'PIDControllerCodeGen/Subsystem1/Subsystem4/Subsystem2/Integral/MATLAB Function2'
//  '<S111>' : 'PIDControllerCodeGen/Subsystem3/MATLAB Function'
//  '<S112>' : 'PIDControllerCodeGen/Subsystem3/pwm_out1'

#endif                                 // RTW_HEADER_PIDControllerCodeGen_h_

//
// File trailer for generated code.
//
// [EOF]
//
