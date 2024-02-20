#include "msg_def.h"
#define PID_OUTPUT_N 5

#define CF_OUTPUT_N 5
#define LPE_OUTPUT_N 5

// #define IMU_RAW_N 3
#define MAG_RAW_N 3
// #define IMU_N 10
#define MAG_N 10
#define ACCEL_N 10
#define GYRO_N 10
#define ACCEL_RAW_N 10
#define GYRO_RAW_N 10
#define BARO_N 3
#define GPS_N 5
#define SBUS_N 5
#define RC_INPUT_N 5
#define ACTUATOR_OUTPUT_N 5
#define ACTUATOR_FEEDBACK_N 5

#define LOG_DISABLE 0u
#define LOG_DIV_1 1u
#define LOG_DIV_2 2u
#define LOG_DIV_3 3u
#define LOG_DIV_5 5U
#define LOG_DIV_10 10u

ringbuffer_typedef<cf_output_typedef> cf_output_msg(CF_OUTPUT_N, "cf_output", LOG_DISABLE);
ringbuffer_typedef<cf_status_typedef> cf_status_msg(CF_OUTPUT_N, "cf_status", LOG_DISABLE);

ringbuffer_typedef<lpe_output_typedef> lpe_output_msg(LPE_OUTPUT_N, "lpe_output", LOG_DISABLE);
ringbuffer_typedef<lpe_output_typedef> lpeLowPass_output_msg(LPE_OUTPUT_N, "lpeLowPass_output", LOG_DISABLE);
ringbuffer_typedef<lpe_status_typedef> lpe_status_msg(LPE_OUTPUT_N, "lpe_status", LOG_DISABLE);

// ringbuffer_typedef<imu_raw_typedef> imu_raw_msg(IMU_RAW_N, "imu_raw", LOG_DISABLE);
ringbuffer_typedef<mag_raw_typedef> mag_raw_msg(MAG_RAW_N, "mag_raw", LOG_DISABLE);
ringbuffer_typedef<mag_typedef> mag_msg(MAG_N,"mag", LOG_DISABLE);
// ringbuffer_typedef<imu_typedef> imu_msg(IMU_N, "imu", LOG_DISABLE);

//currently used for icm42688p
ringbuffer_typedef<accel_raw_typedef> accel_raw_msg(ACCEL_RAW_N,"accel_raw",LOG_DISABLE);
ringbuffer_typedef<accel_typedef> accel_msg(ACCEL_N, "accel",LOG_DISABLE);
ringbuffer_typedef<gyro_raw_typedef> gyro_raw_msg(GYRO_RAW_N,"gyro_raw",LOG_DISABLE);
ringbuffer_typedef<gyro_typedef> gyro_msg(GYRO_N, "gyro", LOG_DISABLE);

ringbuffer_typedef<baro_typedef> baro_msg(BARO_N, "baro", LOG_DISABLE);
ringbuffer_typedef<gps_msg_typedef> gps_msg(GPS_N, "gps", LOG_DISABLE);
ringbuffer_typedef<sbus_packet_t> rc_input_msg(SBUS_N, "rc_input", LOG_DISABLE);
ringbuffer_typedef<actuator_output_typedef> actuator_output_msg(ACTUATOR_OUTPUT_N, "actuator_output", LOG_DISABLE);
ringbuffer_typedef<actuator_output_typedef> aux_actuator_output_msg(ACTUATOR_OUTPUT_N, "aux_actuator_output", LOG_DISABLE);

ringbuffer_typedef<rflysim3d_output_typedef> rflysim3d_output_msg(1,"rflysim3d",LOG_DISABLE);

ringbuffer_typedef<rflypilot_config_typedef> rflypilot_config_msg(1,"config",LOG_DISABLE);

ringbuffer_typedef<scope_data_typedef> controller_scope_msg(20,"controller_debug_data",LOG_DISABLE);
ringbuffer_typedef<scope_data_typedef> att_est_scope_msg(20,"att_est_debug_data",LOG_DISABLE);
ringbuffer_typedef<scope_data_typedef> pos_est_scope_msg(20,"pos_est_debug_data",LOG_DISABLE);
ringbuffer_typedef<scope_data_typedef> system_scope_msg(20,"system_debug_data",LOG_DISABLE);
ringbuffer_typedef<offboard_data_typedef> offboard_msg(5,"offboard_data",LOG_DISABLE);

ringbuffer_typedef<thread_msg_typedef> ctrl_thread_msg(10,"ctrl_thread",LOG_DISABLE);
ringbuffer_typedef<thread_msg_typedef> imu_thread_msg(10,"imu_thread",LOG_DISABLE);
ringbuffer_typedef<thread_msg_typedef> attest_thread_msg(10,"attest_thread",LOG_DISABLE);


ringbuffer_typedef<my_data_typedef> my_data_msg(3, "my_data", LOG_DISABLE);