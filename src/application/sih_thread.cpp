#include "sih_thread.h"

void * thread_sih(void * ptr)
{
  timespec thread_sih_sleep;
  thread_sih_sleep.tv_sec = 0;
  thread_sih_sleep.tv_nsec = 0.88*1000*1000;//0.5ms

  core_bind(SIH_CORE);

  sbus_packet_t _rc_input_msg;//rc_input_msg
  //use imu or, gyro and accel according to fc mode 
  imu_typedef _imu_msg;//imu_msg
  gyro_typedef _gyro_msg;//gyro_msg
  accel_typedef _accel_msg;//accel_msg

  mag_typedef _mag_msg;//mag_msg
  baro_typedef _baro_msg;//baro_msg 
  gps_msg_typedef _gps_msg;//gps_msg

  cf_output_typedef _cf_msg;//cf_output_msg     
  lpe_output_typedef _lpe_msg;//lpe_output_msg

  //at present mpc_output_typedef is used for tranfor control, but in future actuator_output_typedef will be used
  actuator_output_typedef _actuator_output_msg;//actuator_output_msg
  mpc_output_typedef _mpc_output_msg;//mpc_output_msg
  pwm_output_typedef _pwm_output_msg;//pwm_output_msg
  rflysim3d_output_typedef _rflysim3d_output_msg;//rflysim3d_output_msg

  /* Initialize model */
  SIH_Model_initialize();
  while(1){
    /* Set model inputs here */
    // SIH_Model_U._c_out_s;
    pwm_output_msg.read(&_pwm_output_msg);
    memcpy(&SIH_Model_U._c_out_s.pwm, _pwm_output_msg.pwm, sizeof(SIH_Model_U._c_out_s.pwm));
    SIH_Model_U._c_out_s.time_stamp = _pwm_output_msg.timestamp;
    
    /* Step the model */
    SIH_Model_step();

    /* Get model outputs here */
    // SIH_Model_Y._e_cf_s;
    memcpy(&_cf_msg, &SIH_Model_Y._e_cf_s, sizeof(_cf_msg));
    cf_output_msg.publish(&_cf_msg);
    
    // SIH_Model_Y._e_lpe_s;
    memcpy(&_lpe_msg, &SIH_Model_Y._e_lpe_s, sizeof(_lpe_msg));
    lpe_output_msg.publish(&_lpe_msg);

    // SIH_Model_Y._m_accel_s;
    memcpy(&_accel_msg, &SIH_Model_Y._m_accel_s, sizeof(_accel_msg));
    accel_msg.publish(&_accel_msg);

    // SIH_Model_Y._m_gyro_s;
    memcpy(&_gyro_msg, &SIH_Model_Y._m_gyro_s, sizeof(_gyro_msg));
    gyro_msg.publish(&_gyro_msg);

    // SIH_Model_Y._m_mag_s;
    memcpy(&_mag_msg, &SIH_Model_Y._m_mag_s, sizeof(_mag_msg));
    mag_msg.publish(&_mag_msg);

    // SIH_Model_Y._m_baro_s;
    memcpy(&_baro_msg, &SIH_Model_Y._m_baro_s, sizeof(_baro_msg));
    baro_msg.publish(&_baro_msg);

    // SIH_Model_Y._m_gps_s;
    memcpy(&_gps_msg, &SIH_Model_Y._m_gps_s, sizeof(_gps_msg));
    gps_msg.publish(&_gps_msg);

    //SIH_Model_Y.UnRealData;
    memcpy(_rflysim3d_output_msg.unrealdata, SIH_Model_Y.UnRealData, sizeof(_rflysim3d_output_msg.unrealdata));
    rflysim3d_output_msg.publish(&_rflysim3d_output_msg);

    nanosleep(&thread_sih_sleep,NULL);
  }
  
}


void start_sih(void)
{

  bool ret = create_thread("SIH", thread_sih, NULL);

}