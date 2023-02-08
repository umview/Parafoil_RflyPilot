#ifndef _PARAMETER_READ_
#define _PARAMETER_READ_
#include "include.h"
typedef struct
{
	float imu_rate;
	float pid_controller_rate;
	float mpc_rate;
	float actuator_rate;
	float mag_rate;
	float attitude_est_rate;
	float lpe_rate;
	float accel_cutoff;
	float gyro_cutoff;
}config_typedef;
void read_param(void);
extern config_typedef config;


//extern CONF *conf_open(const char *path);

#endif