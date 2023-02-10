#include "log_thread.h"
union log_union_typedef binlog_data;

void * thread_log(void * dir)
{
    timespec thread_log_sleep;
    thread_log_sleep.tv_sec = 0;
    thread_log_sleep.tv_nsec = 10*1000*1000;//2ms	

    cf_output_typedef _att;
    lpe_output_typedef _lpe;
    imu_typedef _imu;
    gps_msg_typedef _gps;
    mag_typedef _mag;
    actuator_output_typedef _actuator_output;
    imu_raw_typedef _imu_raw;

	static float *buff = binlog_data.s.data;
	static int i = 0;

	while(1)
	{

	    buff[0] = 1234;
	    cf_output_msg.read(&_att);
	    lpe_output_msg.read(&_lpe);
	    imu_msg.read(&_imu);
	    gps_msg.read(&_gps);
	    mag_msg.read(&_mag);
	    actuator_output_msg.read(&_actuator_output);
	    imu_raw_msg.read(&_imu_raw);

    for(i = 0; i < 4; i++)
    {
        buff[i+1] = _att.quat[i];
    }
    for(i = 0; i < 3; i++)
    {
        buff[i+5] = _imu.gyro[i];
    }
    for(i = 0; i<4;i++)
    {
      buff[i+8] = 0;
    }
    buff[11+1] = mpc_output_msg.publish_rate_hz;
    for(i = 0; i < 3; i++)buff[i+13] = 0;
    for(i = 0; i < 3; i++)buff[i+16] = 0;
    for(i = 0; i < 4; i++)buff[i+19] = 0;//q_sp
    for(i = 0; i < 3; i++)buff[i+23] = 0;//omega_ref
    for(i = 0; i < 4; i++)buff[i+26] = 0;//pwm
    buff[30] = 0;//cost
    buff[31] = 0;
    buff[32] = 0;
    buff[33] = 0;
    buff[34] = _imu.accel[0];
    buff[35] = _imu.accel[1];
    buff[36] = _imu.accel[2];
    // buff[37] = _pid_output.vel_sp_ned[0];
    // buff[38] = _pid_output.vel_sp_ned[1];
    // buff[39] = _pid_output.vel_sp_ned[2];
    for(i = 0; i< 3; i++)buff[i+37] = _gps.pos_ned[i];
    for(i = 0; i< 3; i++)buff[i+40] = _gps.vel_ned[i];
    for(i = 0; i< 3; i++)buff[i+43] = _lpe.pos_ned[i];
    for(i = 0; i< 3; i++)buff[i+46] = _lpe.vel_ned[i];
    for(i = 0; i< 3; i++)buff[i+49] = _lpe.pos_accel_body[i];
    for(i = 0; i< 3; i++)buff[i+52] = _lpe.accel_bias[i];
    buff[55] = _att.roll;
    buff[56] = _att.pitch;
    buff[57] = _att.yaw;
    for(i = 0; i< 3; i++)buff[i+58] = 0;
    for(i = 0; i< 3; i++)buff[i+61] = 0;
    for(i = 0; i< 3; i++)buff[i+64] = 0;
    buff[69] = cf_output_msg.publish_rate_hz;
    buff[70] = lpe_output_msg.publish_rate_hz;
    buff[71] = imu_msg.publish_rate_hz;
    buff[72] = actuator_output_msg.publish_rate_hz;
    buff[73] = gyro_msg.publish_rate_hz;
    buff[74] = mag_msg.publish_rate_hz;
    buff[78] = (float)icm20689_delay.delay_ns_output;
    buff[79] = (float)attitude_est_delay.delay_ns_output;

	binlog_write((char*)dir,binlog_data.buff);
    nanosleep(&thread_log_sleep,NULL);

	}

}
void start_log(const char * dir)
{
  bool ret = create_thread("log", thread_log, (void*)dir);
}

