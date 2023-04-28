#include "msg_def.h"
#define PID_OUTPUT_N 5

#define CF_OUTPUT_N 5
#define LPE_OUTPUT_N 5

#define IMU_RAW_N 3
#define MAG_RAW_N 3
#define IMU_N 10
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

ringbuffer_typedef<cf_output_typedef> cf_output_msg(CF_OUTPUT_N, "cf_output", true);
ringbuffer_typedef<lpe_output_typedef> lpe_output_msg(LPE_OUTPUT_N, "lpe_output", true);
ringbuffer_typedef<lpe_output_typedef> lpeLowPass_output_msg(LPE_OUTPUT_N, "lpeLowPass_output", true);

ringbuffer_typedef<imu_raw_typedef> imu_raw_msg(IMU_RAW_N);//, "imu_raw", true);
ringbuffer_typedef<mag_raw_typedef> mag_raw_msg(MAG_RAW_N);//, "mag_raw", true);
ringbuffer_typedef<mag_typedef> mag_msg(MAG_N);//, "mag", true);
ringbuffer_typedef<imu_typedef> imu_msg(IMU_N);//, "imu", true);

//currently used for icm42688p
ringbuffer_typedef<accel_raw_typedef> accel_raw_msg(ACCEL_RAW_N);
ringbuffer_typedef<accel_typedef> accel_msg(ACCEL_N, "accel",true);
ringbuffer_typedef<gyro_raw_typedef> gyro_raw_msg(GYRO_RAW_N);
ringbuffer_typedef<gyro_typedef> gyro_msg(GYRO_N, "gyro", true);

ringbuffer_typedef<baro_typedef> baro_msg(BARO_N, "baro", true);
ringbuffer_typedef<gps_msg_typedef> gps_msg(GPS_N, "gps", true);
ringbuffer_typedef<sbus_packet_t> rc_input_msg(SBUS_N, "rc_input", true);
ringbuffer_typedef<actuator_output_typedef> actuator_output_msg(ACTUATOR_OUTPUT_N, "actuator_output", true);

ringbuffer_typedef<rflysim3d_output_typedef> rflysim3d_output_msg(1);

ringbuffer_typedef<rflypilot_config_typedef> rflypilot_config_msg(1,"config",true);

ringbuffer_typedef<scope_data_typedef> controller_scope_msg(20,"controller_debug_data",true);
ringbuffer_typedef<scope_data_typedef> att_est_scope_msg(20,"att_est_debug_data",true);
ringbuffer_typedef<scope_data_typedef> pos_est_scope_msg(20,"pos_est_debug_data",true);
ringbuffer_typedef<scope_data_typedef> system_scope_msg(20,"system_debug_data",false);
ringbuffer_typedef<offboard_data_typedef> offboard_msg(5,"offboard_data",false);
