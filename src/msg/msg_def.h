#ifndef _MSG_DEF_
#define _MSG_DEF_
#include "include.h"
// #pragma pack(push, 4) 

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
  float correct_accel[3];
  float correct_mag[3];
  float magDelay;
  int32_t magDelayIndex;
  bool magDelayStatus;
}cf_status_typedef;
extern ringbuffer_typedef<cf_status_typedef> cf_status_msg;//Q estimator status

typedef struct
{
    uint64_t timestamp;
    double pos_ned[3];
    double vel_ned[3];
    double pos_accel_body[3];
    double accel_bias[3];
}lpe_output_typedef;
extern ringbuffer_typedef<lpe_output_typedef> lpe_output_msg;//local position estimator output message
extern ringbuffer_typedef<lpe_output_typedef> lpeLowPass_output_msg;//local position estimator output message

typedef struct
{
  uint64_t timestamp;
  float xCovariances[9];
  float gpsBeta;
  float baroBeta;
  float gpsInnov[6];
  float baroInnov;
  float gpsAltRef;
  float baroAltRef;
  float gpsDelay;
  int32_t gpsDelayIndex;
  bool gpsDelayStatus;
  bool gpsUpdated;
  bool baroUpdated;
}lpe_status_typedef;
extern ringbuffer_typedef<lpe_status_typedef> lpe_status_msg;//local position estimator status

typedef struct
{
    uint64_t timestamp;
    double pressure;
    double temperature;
}baro_typedef;
extern ringbuffer_typedef<baro_typedef> baro_msg;

// typedef struct
// {
// 	uint64_t timestamp;
// 	float accel[3];
// 	float gyro[3];
// }imu_raw_typedef;
// typedef imu_raw_typedef imu_typedef;
// extern ringbuffer_typedef<imu_raw_typedef> imu_raw_msg;
// extern ringbuffer_typedef<imu_typedef> imu_msg;//(IMU_N);
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
    uint16_t actuator_output[8];
}actuator_output_typedef;
extern ringbuffer_typedef<actuator_output_typedef> actuator_output_msg;//(GPS_N);
extern ringbuffer_typedef<actuator_output_typedef> aux_actuator_output_msg;//(GPS_N);
extern ringbuffer_typedef<actuator_output_typedef> control_output_msg;//用于显示解锁环节前的控制器内部输出，无视是否解锁。

typedef struct
{
    uint64_t timestamp;
    uint8_t unrealdata[200];
}rflysim3d_output_typedef;
extern ringbuffer_typedef<rflysim3d_output_typedef> rflysim3d_output_msg;

typedef struct
{
    uint64_t timestamp;
    float imu_rate;
    float mag_rate;
    float attitude_est_rate;
    float controller_rate;
    float lpe_rate;
    float accel_cutoff_hz;
    float gyro_cutoff_hz;
    bool sys_log_en;
    bool est_log_en;
    bool ctrl_log_en;
    bool sys_scope_en;
    bool est_scope_en;
    bool ctrl_scope_en;
    char scope_ip[20];
    char station_ip[20];
    char log_dir[50];
    validation_mode_typedef validation_mode;
    bool sih_use_real_state;
    scheduler_mode_typedef scheduler_mode;
}rflypilot_config_typedef;
extern ringbuffer_typedef<rflypilot_config_typedef> rflypilot_config_msg;

typedef struct
{
    uint64_t timestamp;
    uint32_t rate_hz;
    float data[SCOPE_DATA_N];
}scope_data_typedef;
typedef struct
{
    uint64_t timestamp;
    uint32_t rate_hz;
    float data[OFFBOARD_DATA_N];
}offboard_data_typedef;
extern ringbuffer_typedef<scope_data_typedef> controller_scope_msg;
extern ringbuffer_typedef<scope_data_typedef> att_est_scope_msg;
extern ringbuffer_typedef<scope_data_typedef> pos_est_scope_msg;
extern ringbuffer_typedef<scope_data_typedef> system_scope_msg;//(20,"system_debug_data",true);
extern ringbuffer_typedef<offboard_data_typedef> offboard_msg;//(20,"system_debug_data",true);


typedef struct
{
    uint64_t timestamp;
    uint64_t data_timestamp;
}thread_msg_typedef;
extern ringbuffer_typedef<thread_msg_typedef> imu_thread_msg;//(1,"imu_thread_msg",LOG_DIV_1);
extern ringbuffer_typedef<thread_msg_typedef> attest_thread_msg;//(1,"attest_thread_msg",LOG_DIV_1);
extern ringbuffer_typedef<thread_msg_typedef> ctrl_thread_msg;//(1,"ctrl_thread_msg",LOG_DIV_1);


typedef struct
{
    uint64_t timestamp;
    float data[2];
}my_data_typedef;
extern ringbuffer_typedef<my_data_typedef> my_data_msg;

#endif