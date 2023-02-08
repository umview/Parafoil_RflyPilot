#include "msg_def.h"
#define PID_OUTPUT_N 5
#define MPC_OUTPUT_N 5

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
#define PWM_N 5
ringbuffer_typedef<pid_output_typedef> pid_output_msg(PID_OUTPUT_N, "pid_output", true);
ringbuffer_typedef<mpc_output_typedef> mpc_output_msg(MPC_OUTPUT_N, "mpc_output", true);

ringbuffer_typedef<cf_output_typedef> cf_output_msg(CF_OUTPUT_N, "cf_output", true);
ringbuffer_typedef<lpe_output_typedef> lpe_output_msg(LPE_OUTPUT_N, "lpe_output", true);

ringbuffer_typedef<imu_raw_typedef> imu_raw_msg(IMU_RAW_N, "imu_raw", true);
ringbuffer_typedef<mag_raw_typedef> mag_raw_msg(MAG_RAW_N, "mag_raw", true);
ringbuffer_typedef<mag_typedef> mag_msg(MAG_N, "mag", true);
ringbuffer_typedef<imu_typedef> imu_msg(IMU_N, "imu", true);

//currently used for icm42688p
ringbuffer_typedef<accel_raw_typedef> accel_raw_msg(ACCEL_RAW_N);
ringbuffer_typedef<accel_typedef> accel_msg(ACCEL_N);
ringbuffer_typedef<gyro_raw_typedef> gyro_raw_msg(GYRO_RAW_N);
ringbuffer_typedef<gyro_typedef> gyro_msg(GYRO_N);

ringbuffer_typedef<baro_typedef> baro_msg(BARO_N);
ringbuffer_typedef<gps_msg_typedef> gps_msg(GPS_N, "gps", true);
ringbuffer_typedef<sbus_packet_t> rc_input_msg(SBUS_N, "rc_input", true);
ringbuffer_typedef<actuator_output_typedef> actuator_output_msg(ACTUATOR_OUTPUT_N, "actuator_output", true);
ringbuffer_typedef<actuator_feedback_typedef> actuator_feedback_msg(ACTUATOR_FEEDBACK_N, "actuator_feedback", true);

ringbuffer_typedef<pwm_output_typedef> pwm_output_msg(PWM_N);
ringbuffer_typedef<rflysim3d_output_typedef> rflysim3d_output_msg(1);
