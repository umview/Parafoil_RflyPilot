#include <string.h>
#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "param_load_block_cgen_wrapper.h"

extern void param_load_block_Start_wrapper(void);
extern void param_load_block_Outputs_wrapper(const uint8_T *param_name,
  real32_T *value,
  int32_T *flag);
extern void param_load_block_Terminate_wrapper(void);
void param_load_block_Start_wrapper_cgen(void)
{
  param_load_block_Start_wrapper();
}

void param_load_block_Outputs_wrapper_cgen(const uint8_T *param_name,
  real32_T *value,
  int32_T *flag)
{
  param_load_block_Outputs_wrapper(param_name,
    value,
    flag);
}

void param_load_block_Terminate_wrapper_cgen(void)
{
  param_load_block_Terminate_wrapper();
}
