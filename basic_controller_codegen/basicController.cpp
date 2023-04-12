//
// File: basicController.cpp
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
#include "basicController.h"
#include "PIDControllerCodeGen.h"

// Model step function
void basicController::step()
{
  // ModelReference generated from: '<Root>/Reference Model' incorporates:
  //   Inport: '<Root>/_c_subs_s'
  //   Inport: '<Root>/_e_cf_s'
  //   Inport: '<Root>/_e_lpe_s'
  //   Inport: '<Root>/_m_gyro_s'
  //   Inport: '<Root>/usec'

  Reference_ModelMDLOBJ1.step(&basicController_U.usec,
    &basicController_U._e_lpe_s.pos_ned[0], &basicController_U._e_lpe_s.vel_ned
    [0], &basicController_U._e_cf_s.quat_data[0],
    &basicController_U._c_subs_s.channels[0],
    &basicController_U._m_gyro_s.time_stamp,
    &basicController_U._m_gyro_s.gyro_data[0],
    &basicController_Y._c_out_s.time_stamp, &basicController_Y._c_out_s.pwm[0],
    &basicController_Y._c_out_s.thrust[0],
    &basicController_Y._s_scope_s.time_stamp,
    &basicController_Y._s_scope_s.rate_hz, &basicController_Y._s_scope_s.data[0]);
}

// Model initialize function
void basicController::initialize()
{
  // Model Initialize function for ModelReference Block: '<Root>/Reference Model' 

  // Set error status pointer for ModelReference Block: '<Root>/Reference Model' 
  Reference_ModelMDLOBJ1.setErrorStatusPointer(rtmGetErrorStatusPointer
    ((&basicController_M)));
  Reference_ModelMDLOBJ1.initialize();

  // SystemInitialize for ModelReference generated from: '<Root>/Reference Model' 
  Reference_ModelMDLOBJ1.init();
}

// Constructor
basicController::basicController() :
  basicController_U(),
  basicController_Y(),
  basicController_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
basicController::~basicController()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
basicController::RT_MODEL_basicController_T * basicController::getRTM()
{
  return (&basicController_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
