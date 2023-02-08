#ifndef _POSITION_ESTIMATOR_API_H_
#define _POSITION_ESTIMATOR_API_H_

#include <stddef.h>
#include <stdio.h>              // This main program uses printf/fflush
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#include "PositionEstimator.h"         // Model header file
#include "include.h"

void start_lpe(void);
void * thread_lpe(void * ptr);
extern class adaptive_delay_typedef lpe_rate;//(0.5,15,10);

#endif
