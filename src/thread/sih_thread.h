#ifndef _SIH_SPI_H_
#define _SIH_SPI_H_

#include <stddef.h>
#include <stdio.h>              // This main program uses printf/fflush
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#include "system.h"
#include "systime.h"

#ifdef __cplusplus
extern "C" {
#endif
    #include "SIH_Model.h"
#ifdef __cplusplus
}
#endif


void start_sih(void);
void * thread_sih(void * ptr);


#endif