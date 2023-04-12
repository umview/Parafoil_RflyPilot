#ifndef _PARAM_LOAD_BLOCK_CGEN_WRAPPER_H_
#define _PARAM_LOAD_BLOCK_CGEN_WRAPPER_H_
#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifdef __cplusplus
#define SFB_EXTERN_C                   extern "C"
#else
#define SFB_EXTERN_C                   extern
#endif

SFB_EXTERN_C void param_load_block_Start_wrapper_cgen(void);
SFB_EXTERN_C void param_load_block_Outputs_wrapper_cgen(const uint8_T
  *param_name,
  real32_T *value,
  int32_T *flag);
SFB_EXTERN_C void param_load_block_Terminate_wrapper_cgen(void);

#undef SFB_EXTERN_C
#endif
