#define POLL_TIME_US 10*1000// 10*1000ns = 10us
#define TIMER_TICK_RATE 2000
#define SCOPE_DATA_N 40
/* Estimator Thread Core */
#define ATT_EST_CORE 2
#define POS_EST_CORE 2

/* Sensor Thread Core */
#define IMU_CORE 2
#define MAG_CORE 2
#define BARO_CORE 2
#define GPS_CORE 2

/* Control Thread Core */
#define SUBS_CORE 2
#define BASIC_CTRL_CORE 2
#define CTRL_CORE 3

/* SIH Thread Core */
#define SIH_CORE 2

/* Serve Thread Core */
#define SCREEN_CORE 1
#define CONSOLE_CORE 1
#define LOG_CORE 1
#define ULOG_CORE 1

/* Rate Div */
#define USING_THREAD_SYNC 0
#define IMU_LPE 1
#define LPE_ATT 1
#define ATT_CTRL 2

#define USE_RFLYPILOT 1
#define USE_ONESHOT_125 1
#define OFFBOARD_DATA_N 20
#define PWM_FREQ (USE_ONESHOT_125==1?1500:400)
#define SERVO_PWM_FREQ 333
#define I2C_BUS_1 "/dev/i2c-1"
#define I2C_BUS_0 "/dev/i2c-10"

#define TASK_SCHEDULE_DEBUG 0