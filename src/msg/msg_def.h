#ifndef _MSG_DEF_
#define _MSG_DEF_
#include "include.h"
typedef struct
{
	uint64_t timestamp;
	// float quat[4];
	// float omega[3];
	float thrust[4];
	float accel_setpoint[3];
	float quat_setpoint[4];
    float vel_sp_ned[3];
    float pos_sp_ned[3];
}pid_output_typedef;
extern ringbuffer_typedef<pid_output_typedef> pid_output_msg;

typedef struct
{
	uint64_t timestamp;
	float thrust[4];
}mpc_output_typedef;
extern ringbuffer_typedef<mpc_output_typedef> mpc_output_msg;
typedef struct
{
    uint64_t timestamp;
    float actuator_feedback[4];
}actuator_feedback_typedef;
extern ringbuffer_typedef<actuator_feedback_typedef> actuator_feedback_msg;

typedef struct
{
    uint64_t timestamp;
    double quat[4];
    double roll;
    double pitch;
    double yaw;
}cf_output_typedef;
extern ringbuffer_typedef<cf_output_typedef> cf_output_msg;//complementory filter output message

typedef struct
{
    uint64_t timestamp;
    double pos_ned[3];
    double vel_ned[3];
    double pos_accel_body[3];
    double accel_bias[3];
}lpe_output_typedef;
extern ringbuffer_typedef<lpe_output_typedef> lpe_output_msg;//local position estimator output message

typedef struct
{
    uint64_t timestamp;
    double pressure;
    double temperature;
}baro_typedef;
extern ringbuffer_typedef<baro_typedef> baro_msg;

typedef struct
{
	uint64_t timestamp;
	float accel[3];
	float gyro[3];
}imu_raw_typedef;
typedef imu_raw_typedef imu_typedef;
extern ringbuffer_typedef<imu_raw_typedef> imu_raw_msg;
extern ringbuffer_typedef<imu_typedef> imu_msg;//(IMU_N);

typedef struct
{
	uint64_t timestamp;
	float accel[3];
}accel_raw_typedef;
typedef accel_raw_typedef accel_typedef;
extern ringbuffer_typedef<accel_raw_typedef> accel_raw_msg;
extern ringbuffer_typedef<accel_typedef> accel_msg;//(ACCEL_N);

typedef struct
{
	uint64_t timestamp;
	float gyro[3];
}gyro_raw_typedef;
typedef gyro_raw_typedef gyro_typedef;
extern ringbuffer_typedef<gyro_raw_typedef> gyro_raw_msg;
extern ringbuffer_typedef<gyro_typedef> gyro_msg;//(GYRO_N);

typedef struct 
{
	uint64_t timestamp;
	float mag[3];
}mag_raw_typedef;
typedef mag_raw_typedef mag_typedef;
extern ringbuffer_typedef<mag_raw_typedef> mag_raw_msg;
extern ringbuffer_typedef<mag_typedef> mag_msg;//(MAG_N);


typedef struct{
    bool updated;
    bool ned_origin_valid;
    bool gps_is_good;
    uint64_t timestamp;
    double lon;
    double lat;
    double lon_origin;
    double lat_origin;
    float yaw_offset;
    float height;
    float vel_ned[3];
    float pos_ned[3];
    float hacc;
    float vacc;
    float sacc;
    float heading;
    float headacc;
    uint8_t numSV;
    uint8_t fixType;
}gps_msg_typedef;
extern ringbuffer_typedef<gps_msg_typedef> gps_msg;//(GPS_N);
typedef struct sbus_packet_t rc_input_typedef;
extern ringbuffer_typedef<sbus_packet_t> rc_input_msg;//(GPS_N);

typedef struct
{
    uint64_t timestamp;
    float actuator_output[4];
}actuator_output_typedef;
extern ringbuffer_typedef<actuator_output_typedef> actuator_output_msg;//(GPS_N);

typedef struct
{
    uint64_t timestamp;
    uint16_t pwm[4];
}pwm_output_typedef;
extern ringbuffer_typedef<pwm_output_typedef> pwm_output_msg;

typedef struct
{
    uint64_t timestamp;
    uint8_t unrealdata[200];
}rflysim3d_output_typedef;
extern ringbuffer_typedef<rflysim3d_output_typedef> rflysim3d_output_msg;


#endif