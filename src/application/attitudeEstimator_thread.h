#ifndef _Q_ATTESTIMATOR_H_
#define _Q_ATTESTIMATOR_H_

#include <stddef.h>
#include <stdio.h>              // This main program uses printf/fflush
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#include "AttitudeEstimator.h"         // Model header file
#include "include.h"

void start_attitudeEstimator(void);
void * thread_attitudeEstimator(void * ptr);
extern class adaptive_delay_typedef attitude_est_delay;//(0.5,15,100);

#endif