#ifndef _BASIC_CONTROLLER_API_H_
#define _BASIC_CONTROLLER_API_H_

#include <stddef.h>
#include <stdio.h>              // This main program uses printf/fflush
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#include "system.h"
#include "systime.h"
#include "include.h"
#include "basicController.h"

void start_basicController(void);
void * thread_basicController(void * ptr);


#endif