//
// File: rt_assert.h
//
// Code generated for Simulink model 'MPCControllerCodeGen_BICOPTER_2020b'.
//
// Model version                  : 5.10
// Simulink Coder version         : 9.8 (R2022b) 13-May-2022
// C/C++ source code generated on : Fri Mar 10 16:52:53 2023
//
#ifndef RTW_HEADER_rt_assert_h_
#define RTW_HEADER_rt_assert_h_

//=========*
//  Asserts *
// =========
#ifndef utAssert
#if defined(DOASSERTS)
#if !defined(PRINT_ASSERTS)
#include <assert.h>
#define utAssert(exp)                  assert(exp)
#else
#include <stdio.h>

static void _assert(char *statement, char *file, int line)
{
  printf("%s in %s on line %d\n", statement, file, line);
}

#define utAssert(_EX)                  ((_EX) ? (void)0 : _assert(#_EX, __FILE__, __LINE__))
#endif

#else
#define utAssert(exp)                                            // do nothing
#endif
#endif
#endif                                 // RTW_HEADER_rt_assert_h_

//
// File trailer for generated code.
//
// [EOF]
//
