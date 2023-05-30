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

    //#include<wiringPi.h>


#ifdef __cplusplus
}
#endif


enum validation_mode_typedef
{
    UNDIFINED_VALIDATION_MODE = 0,
    SIH,
    HIL,
    EXP,
    OFFBOARD,
};

enum scheduler_mode_typedef
{
    UNDIFINED_SCHEDULER_MODE = 0,
    DELAY,
    ADAPTIVE_DELAY,
    TIMER,
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
#include "actuator_fpga.h"
#include "gps_api.h"
#include "scope.h"
#include "scope_thread.h"
#include "screen.h"
#include "console.h"
//#include "console_thread.h"
#include "attitudeEstimator_thread.h"
#include "positionEstimator_thread.h"
#include "usrController_thread.h"
#include "basicController_thread.h"
#include "sih_thread.h"
//#include "gps_thread.h"
//#include "sbus_thread.h"
//#include "icm20689_thread.h"
//#include "icm42688p_thread.h"
//#include "ist8310_thread.h"
//#include "qmc5883l_thread.h"
//#include "ms5611_thread.h"
#include "log_thread.h"
#include "ulog_thread.h"
#include "binlog.h"
#include "system_app.h"
#include "offboard_thread.h"
#endif