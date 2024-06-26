#include "system_app.h"
void * thread_system_app(void * ptr)
{

	cf_output_typedef _att;
	lpe_output_typedef _lpe;
	// imu_typedef _imu;
	gps_msg_typedef _gps;
	mag_typedef _mag;
	baro_typedef _baro;
	actuator_output_typedef _actuator_output;
    gyro_typedef _gyro;
    accel_typedef _accel;
    scope_data_typedef  _system_debug_data;
    sbus_packet_t _rc_input_msg;//rc_input_msg


    int i = 0;
	while(1)
	{
	    cf_output_msg.read(&_att);
	    lpeLowPass_output_msg.read(&_lpe);
	    gps_msg.read(&_gps);
	    mag_msg.read(&_mag);
	    actuator_output_msg.read(&_actuator_output);
	    gyro_msg.read(&_gyro);
	    accel_msg.read(&_accel);
	    baro_msg.read(&_baro);
        rc_input_msg.read(&_rc_input_msg);

	    _system_debug_data.timestamp = get_time_now();


	    _system_debug_data.data[0] = 1234;
	    _system_debug_data.data[1] = get_time_now() / 1e6;
	    _system_debug_data.data[2] = _att.roll;
	    _system_debug_data.data[3] = _att.pitch;
	    _system_debug_data.data[4] = _att.yaw;
	    for(i = 0; i < 3; i++)
	    {
	        _system_debug_data.data[i+5] = _gyro.gyro[i];
	    }
	    for(i = 0; i < 3; i++)
	    {
	        _system_debug_data.data[i+8] = _accel.accel[i];
	    }
	    for(i = 0; i< 3; i++)_system_debug_data.data[i+11] = _lpe.pos_ned[i];
	    for(i = 0; i< 3; i++)_system_debug_data.data[i+14] = _lpe.vel_ned[i];
	    for(i = 0; i< 3; i++)_system_debug_data.data[i+17] = _gps.pos_ned[i];
	    for(i = 0; i< 3; i++)_system_debug_data.data[i+20] = _gps.vel_ned[i];
	    for(i = 0; i < 4; i++)_system_debug_data.data[i+23] = (float)_actuator_output.actuator_output[i];//pwm
	    _system_debug_data.data[27] = actuator_output_msg.publish_rate_hz;
	    _system_debug_data.data[28] = cf_output_msg.publish_rate_hz;
	    _system_debug_data.data[29] = lpe_output_msg.publish_rate_hz;
	    _system_debug_data.data[30] = accel_msg.publish_rate_hz;
	    _system_debug_data.data[31] = mag_msg.publish_rate_hz; 
	    _system_debug_data.data[32] = baro_msg.publish_rate_hz;
	    _system_debug_data.data[33] = _baro.temperature;
	    _system_debug_data.data[34] = _baro.pressure;
	    _system_debug_data.data[35] = _rc_input_msg.channels[0];
	    _system_debug_data.data[36] = _rc_input_msg.channels[1];
	    _system_debug_data.data[37] = _mag.mag[0];
	    _system_debug_data.data[38] = _mag.mag[1];
	    _system_debug_data.data[39] = _mag.mag[2];
	    // _system_debug_data.data[37] = float(sih_model_timestamp);//_mag.mag[0];
	    // _system_debug_data.data[38] = float(sih_timestamp);//_mag.mag[1];
	    // _system_debug_data.data[39] = ((float)sih_model_timestamp - (float)sih_timestamp);//_mag.mag[2];

	    system_scope_msg.publish(&_system_debug_data);  
		//printf("%f %f %f %f\n",actuator_output_msg.publish_rate_hz, cf_output_msg.publish_rate_hz,mag_msg.publish_rate_hz,accel_msg.publish_rate_hz);

		usleep(10000);
	}

}


void start_system_app(void)
{
  bool ret = create_thread("system_app", thread_system_app, NULL);

}