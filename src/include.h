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
    #include "conf-c/conf.h"
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
#include "limit.h"

#include "configure.h"
#include "system.h"
#include "systime.h"
#include "math_function.h"
#include "ringbuffer.h"
#include "msg_def.h"
#include "pca9685.h"
#include "sbus_api.h"
#include "ICM42688P.h"
#include "icm20689.h"
#include "sensor_calibration.h"
#include "ellipsoid_method.h"
#include "ellipsoid_method_types.h"
#include "parameter_read.h"
#include "test_time.h"
#include "ist8310.h"
#include "QMC5883L.h"
#include "ms5611.h"
#include "gps_api.h"
#include "attitudeEstimator_thread.h"
#include "positionEstimator_thread.h"
#include "usrController_thread.h"

#endif