#ifndef ELLIPSOID_METHOD_H
#define ELLIPSOID_METHOD_H
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ellipsoid_method_types.h"

extern void ellipsoid_method_initialize(void);
extern void ellipsoid_method_step1(double x, double y, double z, double A[49]);
extern void ellipsoid_method_step2(const double A[49], double *x_scale, double
  *y_scale, double *z_scale, double *x_offset, double *y_offset, double
  *z_offset);
extern void ellipsoid_method_terminate(void);

#endif
