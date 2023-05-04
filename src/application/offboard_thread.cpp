#include "offboard_thread.h"

class udp_recv_class offboard_udp_recv;

void * thread_offboard(void * ptr)
{


	offboard_udp_recv.init("192.168.199.152",2222);

    actuator_output_typedef _actuator_output_msg;//actuator_output_msg
    actuator_output_typedef _aux_actuator_output_msg;//actuator_output_msg

	offboard_data_typedef _offboard_data;
	while(1)
	{
	    // cf_output_msg.read(&_att);
	    // lpeLowPass_output_msg.read(&_lpe);
	    // gps_msg.read(&_gps);
	    // mag_msg.read(&_mag);
	    // actuator_output_msg.read(&_actuator_output);
	    // gyro_msg.read(&_gyro);
	    // accel_msg.read(&_accel);
	    // baro_msg.read(&_baro);
     //    rc_input_msg.read(&_rc_input_msg);

	    _offboard_data.timestamp = get_time_now();
	    offboard_udp_recv.udp_recv((uint8_t *)&_offboard_data.data[0], sizeof(_offboard_data.data));
	    offboard_msg.publish(&_offboard_data);
        _actuator_output_msg.timestamp = _offboard_data.timestamp;
        _aux_actuator_output_msg.timestamp = _offboard_data.timestamp;
	    float pwm_output[8];
	    float aux_pwm_output[8];
        for(int i = 0; i < 8; i++)
        {
            if(USE_ONESHOT_125 == 1)
            {
                pwm_output[i] = ((float)_offboard_data.data[i])/8;
            }else{
                pwm_output[i] = ((float)_offboard_data.data[i]);
            }
                pwm_output[i] =1500;

        }

        for(int i = 8; i < 16; i++)
        {

            aux_pwm_output[i-8] = 1500;//((float)_offboard_data.data[i]);

        }

        for(int i = 0; i < 8; i++)
        {
			_actuator_output_msg.actuator_output[i] = pwm_output[i];
			_actuator_output_msg.actuator_output[i] = aux_pwm_output[i];
        }
    	  actuator_output_msg.publish(&_actuator_output_msg);
    	  aux_actuator_output_msg.publish(&_aux_actuator_output_msg);

          //pca9685_dev.updatePWM(pwm_output,8);
          pca9685_dev_aux.updatePWM(aux_pwm_output,8);
          // printf("actuator : %f %f %f %f\n", _pwm[0],_pwm[1],_pwm[2],_pwm[3]);


		usleep(10000);
		//printf("%lld %f %f %f %f\n", _offboard_data.timestamp, _offboard_data.data[0], _offboard_data.data[1],
		//												   _offboard_data.data[2], _offboard_data.data[2]);
	}

}


void start_offboard(void)
{
  bool ret = create_thread("offboard", thread_offboard, NULL);

}