#include "sih_thread.h"

void * thread_sih(void * ptr)
{
  timespec thread_sih_sleep;
  thread_sih_sleep.tv_sec = 0;
  thread_sih_sleep.tv_nsec = 900*1000;

  core_bind(SIH_CORE);

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

  //at present mpc_output_typedef is used for tranfor control, but in future actuator_output_typedef will be used
  actuator_output_typedef _actuator_output_msg;//actuator_output_msg

  rflysim3d_output_typedef _rflysim3d_output_msg;//rflysim3d_output_msg

  rflypilot_config_typedef _config;//rflypilot_config_msg
  rflypilot_config_msg.read(&_config);

  /* Initialize model */
  SIH_Model_initialize();
  #if USING_THREAD_SYNC
	char count_for_lpe = 0;
	#endif
  while(1){
    /* Set model inputs here */
    // SIH_Model_U._c_out_s;
    actuator_output_msg.read(&_actuator_output_msg);
    memcpy(&SIH_Model_U._c_out_s.pwm, _actuator_output_msg.actuator_output, sizeof(SIH_Model_U._c_out_s.pwm));
    SIH_Model_U._c_out_s.time_stamp = _actuator_output_msg.timestamp;
    
    /* Step the model */
    SIH_Model_step();

    //SIH_Model_Y.UnRealData;
    memcpy(_rflysim3d_output_msg.unrealdata, SIH_Model_Y.UnRealData, sizeof(_rflysim3d_output_msg.unrealdata));
    rflysim3d_output_msg.publish(&_rflysim3d_output_msg);

    /* Get model outputs here */
    if(_config.sih_use_real_state)
    {
      // SIH_Model_Y._e_cf_s;
      memcpy(&_cf_msg, &SIH_Model_Y._e_cf_s, sizeof(_cf_msg));
      cf_output_msg.publish(&_cf_msg);
      
      // SIH_Model_Y._e_lpe_s;
      memcpy(&_lpe_msg, &SIH_Model_Y._e_lpe_s, sizeof(_lpe_msg));
      lpe_output_msg.publish(&_lpe_msg);

      lpeLowPass_output_msg.publish(&_lpe_msg);
    }

    // SIH_Model_Y._m_gps_s;
    uint64_t gps_timestamp_old = _gps_msg.timestamp;
    memcpy(&_gps_msg, &SIH_Model_Y._m_gps_s, sizeof(_gps_msg));
    if(_gps_msg.timestamp>gps_timestamp_old)gps_msg.publish(&_gps_msg);

    // SIH_Model_Y._m_baro_s;
    uint64_t baro_timestamp_old = _baro_msg.timestamp;
    memcpy(&_baro_msg, &SIH_Model_Y._m_baro_s, sizeof(_baro_msg));
    if(_baro_msg.timestamp > baro_timestamp_old)baro_msg.publish(&_baro_msg);

    // SIH_Model_Y._m_mag_s;
    uint64_t mag_timestamp_old = _mag_msg.timestamp;
    memcpy(&_mag_msg, &SIH_Model_Y._m_mag_s, sizeof(_mag_msg));
    if(_mag_msg.timestamp > mag_timestamp_old)mag_msg.publish(&_mag_msg);

    // SIH_Model_Y._m_accel_s;
    memcpy(&_accel_msg, &SIH_Model_Y._m_accel_s, sizeof(_accel_msg));
    accel_msg.publish(&_accel_msg);

    // SIH_Model_Y._m_gyro_s;
    memcpy(&_gyro_msg, &SIH_Model_Y._m_gyro_s, sizeof(_gyro_msg));
    gyro_msg.publish(&_gyro_msg);

    #if USING_THREAD_SYNC
		if(count_for_lpe == IMU_LPE)
		{
		pthread_mutex_lock(&mutex_imu2lpe);  
		pthread_cond_signal(&cond_imu2lpe);   
		pthread_mutex_unlock(&mutex_imu2lpe);
		count_for_lpe = 0;
		}
		count_for_lpe++;
		#endif

    nanosleep(&thread_sih_sleep,NULL);
  }
  
}


void start_sih(void)
{

  bool ret = create_thread("SIH", thread_sih, NULL);

}