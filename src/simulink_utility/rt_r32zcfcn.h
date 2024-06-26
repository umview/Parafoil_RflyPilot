//
// File: rt_r32zcfcn.h
//
// Code generated for Simulink model 'MPCControllerCodeGen_Vz'.
//
// Model version                  : 1.142
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Wed Mar  1 20:58:25 2023
//
#ifndef RTW_HEADER_rt_r32zcfcn_h_
#define RTW_HEADER_rt_r32zcfcn_h_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "solver_zc.h"
#ifndef slZcHadEvent
#define slZcHadEvent(ev, zcsDir)       (((ev) & (zcsDir)) != 0x00 )
#endif

#ifndef slZcUnAliasEvents
#define slZcUnAliasEvents(evL, evR)    ((((slZcHadEvent((evL), (SL_ZCS_EVENT_N2Z)) && slZcHadEvent((evR), (SL_ZCS_EVENT_Z2P))) || (slZcHadEvent((evL), (SL_ZCS_EVENT_P2Z)) && slZcHadEvent((evR), (SL_ZCS_EVENT_Z2N)))) ? (SL_ZCS_EVENT_NUL) : (evR)))
#endif

#ifdef __cplusplus

extern "C"
{

#endif

  extern ZCEventType rt_R32ZCFcn(ZCDirection zcDir, ZCSigState *prevZc, real32_T
    currValue);

#ifdef __cplusplus

}                                      // extern "C"

#endif
#endif                                 // RTW_HEADER_rt_r32zcfcn_h_

//
// File trailer for generated code.
//
// [EOF]
//
