#ifndef _SENSOR_CALIBRATION_
#define _SENSOR_CALIBRATION_
#include "include.h"
#define calibration_delay_us 5000
#define mag_calibration_delay_us 10000

#define calibration_debug 1

void start_calibration(char *inputs);

typedef enum{
	CAL_DISABLE = 0,
	CAL_ACCEL,
	CAL_GYRO,
	CAL_MAG,
	CAL_BARO,
	CAL_RC,
	SAVE,
	LOAD,
}calibration_mode_typedef;
typedef enum
{
	NZ_UP,
	PZ_UP,
	NY_UP,
	PY_UP,
	NX_UP,
	PX_UP,
}orientation_typedef;
typedef struct 
{
	double accel_scale[3];
	double accel_offset[3];
	double gyro_scale[3];
	double gyro_offset[3];
	double mag_scale[3];
	double mag_offset[3];
}calib_data_typedef;
class calibration_typedef
{
public:
	calibration_mode_typedef calib_mode;
	calib_data_typedef calib_data;
	orientation_typedef orientation;
	bool calibration_busy_flag;
	bool sensor_calib_enable;
	bool rc_calib_enable;
	calibration_typedef(void);
	void init(void);
	void run_calibration(void);
	bool calib_accel(double G);
	bool calib_gyro(void);
	bool calib_mag(float mag_norm);
	bool calib_baro(void);
	bool calib_rc(void);
	bool orientation_check(float accel[3], float gyro[3],orientation_typedef _dir);
	void write_config(calib_data_typedef config);
	void apply_accel_calibration(float accel[3],float accel_calib[3]);
	void apply_gyro_calibration(float gyro[3],float gyro_calib[3]);
	void apply_mag_calibration(float mag[3],float mag_calib[3]);
	void load_param(calib_data_typedef *config);
	void calibration_file_check_and_load(void);
};
extern class calibration_typedef calibration;
#endif