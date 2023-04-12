//
// File: basicController.h
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
#ifndef RTW_HEADER_basicController_h_
#define RTW_HEADER_basicController_h_
#include "rtwtypes.h"
#include "basicController_types.h"
#include "PIDControllerCodeGen.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  ((const char_T **)(&((rtm)->errorStatus)))
#endif

// Class declaration for model basicController
class basicController final
{
  // public data and function members
 public:
  // External inputs (root inport signals with default storage)
  struct ExtU_basicController_T {
    uint64_T usec;                     // '<Root>/usec'
    e_lpe_s _e_lpe_s;                  // '<Root>/_e_lpe_s'
    e_cf_s _e_cf_s;                    // '<Root>/_e_cf_s'
    c_subs_s _c_subs_s;                // '<Root>/_c_subs_s'
    m_gyro_s _m_gyro_s;                // '<Root>/_m_gyro_s'
    m_accel_s _m_accel_s;              // '<Root>/_m_accel_s'
    m_mag_s _m_mag_s;                  // '<Root>/_m_mag_s'
    m_gps_s _m_gps_s;                  // '<Root>/_m_gps_s'
    m_baro_s _m_baro_s;                // '<Root>/_m_baro_s'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_basicController_T {
    c_out_s _c_out_s;                  // '<Root>/_c_out_s'
    s_scope_s _s_scope_s;              // '<Root>/_s_scope_s'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_basicController_T {
    const char_T *errorStatus;
  };

  // Copy Constructor
  basicController(basicController const&) = delete;

  // Assignment Operator
  basicController& operator= (basicController const&) & = delete;

  // Move Constructor
  basicController(basicController &&) = delete;

  // Move Assignment Operator
  basicController& operator= (basicController &&) = delete;

  // Real-Time Model get method
  basicController::RT_MODEL_basicController_T * getRTM();

  // External inputs
  ExtU_basicController_T basicController_U;

  // External outputs
  ExtY_basicController_T basicController_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  basicController();

  // Destructor
  ~basicController();

  // private data and function members
 private:
  // model instance variable for '<Root>/Reference Model'
  PIDControllerCodeGen Reference_ModelMDLOBJ1;

  // Real-Time Model
  RT_MODEL_basicController_T basicController_M;
};

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
//  '<Root>' : 'basicController'

#endif                                 // RTW_HEADER_basicController_h_

//
// File trailer for generated code.
//
// [EOF]
//
