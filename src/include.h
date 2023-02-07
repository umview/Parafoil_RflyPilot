#ifndef _CONFIG_H_
#define _CONFIG_H_
//#define _GNU_SOURCE

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif

    #include <stdio.h>
    #include <string.h>
    #include <errno.h>

    #include <stdbool.h>

    #include <sys/time.h>
    #include <sched.h>
    #include <assert.h>
    #include <time.h>
    #include <signal.h>

    #include <stdlib.h>
    #include <pthread.h>
    #include <sched.h>
    #include <sys/types.h>
    #include <unistd.h>
    //#include "conf-c/conf.h"
    #include <termios.h>
    #include <string.h>
    #include <sys/ioctl.h>    
    #include <linux/spi/spidev.h>
    #include <fcntl.h>
    #include <dirent.h>
#ifdef __cplusplus
}
#endif

enum fc_validation_mode_typedef
{
    HIL,
    HIL2,
    EXP,
    EXP2,
    SIH,
};

#include "systime.h"



#endif