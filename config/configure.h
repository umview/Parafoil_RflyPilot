#define POLL_TIME_US 10*1000// 10*1000ns = 10us
#define TIMER_TICK_RATE 2000
#define SCOPE_DATA_N 40
#define ATT_EST_CORE 2
#define POS_EST_CORE 2
#define IMU_CORE 2
#define CTRL_CORE 3
#define SIH_CORE 1
#define MAG_CORE 2
#define BASIC_CTRL_CORE 2
#define USE_RFLYPILOT 0
#define USE_ONESHOT_125 0
#define OFFBOARD_DATA_N 20
#define PWM_FREQ (USE_ONESHOT_125==1?1500:400)
#define SERVO_PWM_FREQ 333
#define I2C_BUS_1 "/dev/i2c-1"
#define I2C_BUS_0 "/dev/i2c-10"