#ifndef _TEST_TIME_H_
#define _TEST_TIME_H_

// #include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#ifndef TESTMODE
	#include "configure.h"
#else
uint64_t get_time_now(void);
extern uint64_t get_time_now(void);
#endif





#endif