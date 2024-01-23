#include "usrController_thread.h"
class adaptive_delay_typedef controller_delay(0.5,15,1000);

static usrController usrController_Obj;

void * thread_usrController(void * ptr)
{
    timespec thread_usrController_sleep;
    thread_usrController_sleep.tv_sec = 0;
    thread_usrController_sleep.tv_nsec = 2.5*1000*1000;//2ms

    /* define input struct */
    core_bind(CTRL_CORE);

    /* usrController init */
    usrController_Obj.initialize();
    int cont = 500;
    int i = 0;
    sbus_packet_t _rc_input_msg;//rc_input_msg
    //use imu or, gyro and accel according to fc mode 
    // imu_typedef _imu_msg;//imu_msg
    gyro_typedef _gyro_msg;//gyro_msg
    accel_typedef _accel_msg;//accel_msg

    mag_typedef _mag_msg;//mag_msg
    baro_typedef _baro_msg;//baro_msg 
    gps_msg_typedef _gps_msg;//gps_msg

    cf_output_typedef _cf_msg;//cf_output_msg     
    lpe_output_typedef _lpe_msg;//lpe_output_msg
    lpe_output_typedef _lpeLowPass_msg;//lpeLowPass_output_msg

    //actuator_output_typedef will be used
    actuator_output_typedef _actuator_output_msg;//actuator_output_msg
    actuator_output_typedef _aux_actuator_output_msg;//actuator_output_msg

    scope_data_typedef _controller_debug;

    //rc_input_typedef _rc_input;

    rflypilot_config_typedef _config_msg;
    rflypilot_config_msg.read(&_config_msg);

    thread_msg_typedef _ctrl;


    while (1)
    { 
      #if USING_THREAD_SYNC
      pthread_mutex_lock(&mutex_att2ctrl);  
      pthread_cond_wait(&cond_att2ctrl, &mutex_att2ctrl);  
      pthread_mutex_unlock(&mutex_att2ctrl);  
      #endif
      //time_stamp
      usrController_Obj.usrController_U.time_stamp = get_time_now();
      //rc
      rc_input_msg.read(&_rc_input_msg);
      memcpy(&usrController_Obj.usrController_U._c_subs_s, &_rc_input_msg, sizeof(usrController_Obj.usrController_U._c_subs_s));
      // usrController_Obj.usrController_U._c_subs_s.channels[0] = 1500;
      // usrController_Obj.usrController_U._c_subs_s.channels[1] = 1500;
      usrController_Obj.usrController_U._c_subs_s.channels[2] = _rc_input_msg.channels[3];
      usrController_Obj.usrController_U._c_subs_s.channels[3] = _rc_input_msg.channels[2];
      //set gyro and accel
      //EXP2 use real pilotpi with real sensor
      // if(fc_api.valid_mode == EXP2 || fc_api.valid_mode == SIH){
          gyro_msg.read(&_gyro_msg);
          accel_msg.read(&_accel_msg);
        //   printf("info read   : timestamp: %lld, accel x: %f, accel y: %f, accel z: %f\n\n", _accel_msg.timestamp,_accel_msg.accel[0], _accel_msg.accel[1], _accel_msg.accel[2]);
          memcpy(&usrController_Obj.usrController_U._m_gyro_s,&_gyro_msg,sizeof(usrController_Obj.usrController_U._m_gyro_s));
          memcpy(&usrController_Obj.usrController_U._m_accel_s,&_accel_msg,sizeof(usrController_Obj.usrController_U._m_accel_s));
      // }else{
      //     imu_msg.read(&_imu_msg);
      //     memcpy(&usrController_Obj.usrController_U._m_accel_s.accel_data, &_imu_msg.accel, sizeof(usrController_Obj.usrController_U._m_accel_s.accel_data));
      //     memcpy(&usrController_Obj.usrController_U._m_gyro_s.gyro_data, &_imu_msg.gyro, sizeof(usrController_Obj.usrController_U._m_gyro_s.gyro_data));
      //     usrController_Obj.usrController_U._m_accel_s.time_stamp = _imu_msg.timestamp;
      //     usrController_Obj.usrController_U._m_gyro_s.time_stamp = _imu_msg.timestamp;
      // }
      // mag check pass
      mag_msg.read(&_mag_msg);
      memcpy(&usrController_Obj.usrController_U._m_mag_s, &_mag_msg, sizeof(usrController_Obj.usrController_U._m_mag_s));
      // baro check pass
      baro_msg.read(&_baro_msg);
      memcpy(&usrController_Obj.usrController_U._m_baro_s, &_baro_msg, sizeof(usrController_Obj.usrController_U._m_baro_s));
      // gps check pass
      gps_msg.read(&_gps_msg);
      memcpy(&usrController_Obj.usrController_U._m_gps_s, &_gps_msg, sizeof(usrController_Obj.usrController_U._m_gps_s));
      // cf check pass
      cf_output_msg.read(&_cf_msg);
      uint64_t att_time_use = get_time_now() - _cf_msg.timestamp;
      // if(TASK_SCHEDULE_DEBUG)
      // {
      //   _ctrl.timestamp = get_time_now();
      //   _ctrl.data_timestamp = _cf_msg.timestamp;
      //   ctrl_thread_msg.publish(&_ctrl);
      // }

      memcpy(&usrController_Obj.usrController_U._e_cf_s, &_cf_msg, sizeof(usrController_Obj.usrController_U._e_cf_s));
      //lpe check pass
      lpeLowPass_output_msg.read(&_lpeLowPass_msg);
      // memcpy(&usrController_Obj.usrController_U._e_lpe_s, &_lpeLowPass_msg, sizeof(usrController_Obj.usrController_U._e_lpe_s));
      // uint64_t lpe_time_use = get_time_now() - _lpeLowPass_msg.timestamp;
    //   printf("info: poslp ned %f, %f, %f\n", _lpeLowPass_msg.pos_ned[0], _lpeLowPass_msg.pos_ned[1], _lpeLowPass_msg.pos_ned[2]);
    //   printf("info: vellp ned %f, %f, %f\n\n", _lpeLowPass_msg.vel_ned[0], _lpeLowPass_msg.vel_ned[1], _lpeLowPass_msg.vel_ned[2]);
      lpe_output_msg.read(&_lpe_msg);
      memcpy(&usrController_Obj.usrController_U._e_lpe_s, &_lpe_msg, sizeof(usrController_Obj.usrController_U._e_lpe_s));
    //     printf("info: pos ned %f, %f, %f\n", _lpe_msg.pos_ned[0], _lpe_msg.pos_ned[1], _lpe_msg.pos_ned[2]);
    //     printf("info: vel ned %f, %f, %f\n\n", _lpe_msg.vel_ned[0], _lpe_msg.vel_ned[1], _lpe_msg.vel_ned[2]);
      //run step
      usrController_Obj.step();
      
      //publish debug
      // for(int i = 0; i < 4; i++)
      // {
      //   _controller_debug.data[i] = _actuator_output_msg.actuator_output[i];
      // }
      memcpy(&_controller_debug, &usrController_Obj.usrController_Y._s_scope_s, sizeof(scope_data_typedef));
      _controller_debug.timestamp = usrController_Obj.usrController_Y._s_scope_s.time_stamp;
      //printf("%f %f %f %f\n", _controller_debug.data[0], _controller_debug.data[1], _controller_debug.data[2],_controller_debug.data[3]);
      controller_scope_msg.publish(&_controller_debug);
      
      //publish _actuator_output_msg for Simulink Display and SIH mode
      memcpy(&_actuator_output_msg.actuator_output, &usrController_Obj.usrController_Y._c_out_s.pwm, sizeof(_actuator_output_msg.actuator_output));
      _actuator_output_msg.timestamp = usrController_Obj.usrController_Y._c_out_s.time_stamp;
	  control_output_msg.publish(&_actuator_output_msg);//publish control output disregard disarm;  kechenxu 2023 12 09   
      // printf("pwm out is: %d %d %d %d\n",usrController_Obj.usrController_Y._c_out_s.pwm[0],usrController_Obj.usrController_Y._c_out_s.pwm[1],usrController_Obj.usrController_Y._c_out_s.pwm[2],usrController_Obj.usrController_Y._c_out_s.pwm[3]);
      //fail safe
      float pwm_output[8] = {0};
      float pwm_aux_output[8] = {0};
      if(((get_time_now() - _rc_input_msg.timestamp) > 8e5) || (_rc_input_msg.failsafe) || (_rc_input_msg.frameLost) || _rc_input_msg.channels[5] < 1200) 
      {
        // if(cont == 500)printf("remote controller signal loss\n");
        for(i = 0; i<8; i++)
        {
            if(USE_ONESHOT_125 == 1)
            {
                pwm_output[i] = 125.f;
            }
            else
            {
                pwm_output[i] = 1000.f;
            }
            //usrController_Obj.usrController_Y._c_out_s.pwm[i] = 1000;
            pwm_aux_output[i] = 1500;
        }
        const uint16_t disarm_pwm[8] = {1000U,1000U,1000U,1000U,1000U,1000U,1000U,1000U};
        memcpy(&_actuator_output_msg.actuator_output, &disarm_pwm,sizeof(_actuator_output_msg.actuator_output));
      }else{
        for(int i = 0; i < 8; i++)
        {
            if(USE_ONESHOT_125 == 1)
            {
                pwm_output[i] = ((float)_actuator_output_msg.actuator_output[i])/8;
            }else{
                pwm_output[i] = ((float)_actuator_output_msg.actuator_output[i]);
            }
            pwm_aux_output[i] = ((float)_aux_actuator_output_msg.actuator_output[i]);
        }
      }
      actuator_output_msg.publish(&_actuator_output_msg);
      aux_actuator_output_msg.publish(&_aux_actuator_output_msg);
      // static uint16_t pwm_cnt = 0;
      //set pwm
      if(_config_msg.validation_mode ==  EXP || _config_msg.validation_mode ==  HIL)
      {
        //   debug_io.toggle_io();
        //   if(debug_io.value == 1)pwm_cnt = 2000;
        //   else pwm_cnt = 50;
        // for(int i = 0; i < 8; i++)
        // {
        //     if(USE_ONESHOT_125 == 1)
        //     {
        //         pwm_output[i] = ((float)(pwm_cnt))/8;
        //     }else{
        //         pwm_output[i] = ((float)(pwm_cnt));
        //     }
        // }
          pca9685_dev.updatePWM(pwm_output,4);
          if(_config_msg.validation_mode == EXP) pca9685_dev_aux.updatePWM(pwm_aux_output,8);
          // printf("actuator : %f %f %f %f\n", pwm_output[0],pwm_output[1],pwm_output[2],pwm_output[3]);
      }

  
      //print debug info 
      cont--;
      if (cont<1)
      {   
          cont = 1600;
          // printf("thrust: %f %f %f %f ", _mpc_output_msg.thrust[0], _mpc_output_msg.thrust[1], _mpc_output_msg.thrust[2], _mpc_output_msg.thrust[3]);
          // printf("pwm: %d %d %d %d ",_actuator_output_msg.actuator_output[0], _actuator_output_msg.actuator_output[1], _actuator_output_msg.actuator_output[2], _actuator_output_msg.actuator_output[3]);
          // printf("gyro: %f %f %f ",usrController_Obj.usrController_U._m_gyro_s.gyro_data[0], usrController_Obj.usrController_U._m_gyro_s.gyro_data[1], usrController_Obj.usrController_U._m_gyro_s.gyro_data[2]);
          // printf("quat: %f %f %f %f ",usrController_Obj.usrController_U._e_cf_s.quat_data[0], usrController_Obj.usrController_U._e_cf_s.quat_data[1], usrController_Obj.usrController_U._e_cf_s.quat_data[2], usrController_Obj.usrController_U._e_cf_s.quat_data[3]);
          // printf("vel: %f %f %f ", usrController_Obj.usrController_U._e_lpe_s.vel_ned[0], usrController_Obj.usrController_U._e_lpe_s.vel_ned[1], usrController_Obj.usrController_U._e_lpe_s.vel_ned[2]);
          // printf("sbus: %d %d %d %d \n", usrController_Obj.usrController_U._c_subs_s.channels[0], usrController_Obj.usrController_U._c_subs_s.channels[1], usrController_Obj.usrController_U._c_subs_s.channels[2], usrController_Obj.usrController_U._c_subs_s.channels[3]);
          // printf("info: ATT time interval from published to used: %lld us\n\n", att_time_use);
      }
      #if USING_THREAD_SYNC

      #else
          //nanosleep(&thread_usrController_sleep,NULL);
          delay_us_combined((uint64_t)(1000000.f / _config_msg.controller_rate),&scheduler.controller_flag,&controller_delay);
      #endif

    }

    return NULL;
}

void start_usrController(void)
{
  //   int rc;
  //   pthread_t thr_usrController;
  //   if(rc = pthread_create(&thr_usrController, NULL, thread_usrController, NULL))
  //   {
		// printf("usrController thread cretated failed %d \n", rc);
  //   }
  //   printf("usrController thread created with process pid : %d\n", (int)getpid()); 

  bool ret = create_thread("Controller", thread_usrController, NULL);


}