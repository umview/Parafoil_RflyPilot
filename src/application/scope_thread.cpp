#include "scope_thread.h"
class scope_class system_scope;
class scope_class rflysim3dDP;
class scope_class controller_scope;
class scope_class att_est_scope;
class scope_class pos_est_scope;
#define scope_debug 0
void * thread_system_scope(void * ptr)
{
  //piHiPri(0);
 uint64_t t0 = 0;
  uint64_t t1 = 0;
  int i = 0;
  //get_hight_Pri(1);

  char *addr = (char*)ptr;
  system_scope.init(addr, 3333);

  rflysim3dDP.init(addr, 20010);
  rflysim3d_output_typedef _rflysim3d_output_msg;//rflysim3d_output_msg
	cf_output_typedef _att;
	lpe_output_typedef _lpe;
	imu_typedef _imu;
	gps_msg_typedef _gps;
	mag_typedef _mag;
	actuator_output_typedef _actuator_output;
	imu_raw_typedef _imu_raw;
    gyro_typedef _gyro;
    accel_typedef _accel;
  float buff[80];
  buff[0] = 1234;
  while(1)
  {

    cf_output_msg.read(&_att);
    lpe_output_msg.read(&_lpe);
    imu_msg.read(&_imu);
    gps_msg.read(&_gps);
    mag_msg.read(&_mag);
    actuator_output_msg.read(&_actuator_output);
    imu_raw_msg.read(&_imu_raw);
    gyro_msg.read(&_gyro);
    accel_msg.read(&_accel);
    for(i = 0; i < 4; i++)
    {
        buff[i+1] = _att.quat[i];
    }
    for(i = 0; i < 3; i++)
    {
        buff[i+5] = _gyro.gyro[i];
    }
    for(i = 0; i<4;i++)
    {
      buff[i+8] = 0;
    }
    buff[11+1] = 0;
    for(i = 0; i < 3; i++)buff[i+13] = 0;
    for(i = 0; i < 3; i++)buff[i+16] = 0;
    for(i = 0; i < 4; i++)buff[i+19] = 0;//q_sp
    for(i = 0; i < 3; i++)buff[i+23] = 0;//omega_ref
    for(i = 0; i < 4; i++)buff[i+26] = (float)_actuator_output.actuator_output[i];//pwm
    buff[30] = 0;//cost
    buff[31] = 0;
    buff[32] = 0;
    buff[33] = get_time_now() / 1e6;
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
    buff[67] = 0;
    buff[68] = actuator_output_msg.publish_rate_hz;
    buff[69] = cf_output_msg.publish_rate_hz;
    buff[70] = lpe_output_msg.publish_rate_hz;
    buff[71] = imu_msg.publish_rate_hz;
    buff[72] = actuator_output_msg.publish_rate_hz;
    buff[73] = gyro_msg.publish_rate_hz;
    buff[74] = mag_msg.publish_rate_hz;
    buff[75] = 0;
    buff[76] = 0;
    buff[77] = 0;
    buff[78] = 0;
    buff[79] = 0;
    //for(i = 0; i< 3; i++)buff[i+75] = _mag.mag[i];
    //for(i = 0; i< 3; i++)buff[i+75] = _imu_raw.gyro[i];
    
	system_scope.udp_send((uint8_t*)(buff),sizeof(buff));

    rflysim3d_output_msg.read(&_rflysim3d_output_msg);
    rflysim3dDP.udp_send(_rflysim3d_output_msg.unrealdata,200);

    usleep(10000);
  }
}
void * thread_controller_scope(void * ptr)
{
  char *addr = (char*)ptr;   
  controller_scope.init(addr,3334);
  scope_data_typedef _controller_debug;
  while(1)
  {
    controller_scope_msg.read(&_controller_debug);
    controller_scope.udp_send((uint8_t*)(_controller_debug.data),sizeof(_controller_debug.data));

    usleep(10000);

  }
}
void * thread_att_est_scope(void * ptr)
{
  char *addr = (char*)ptr;    
  att_est_scope.init(addr,3335);
  scope_data_typedef _att_est_debug;
  while(1)
  {
    att_est_scope_msg.read(&_att_est_debug);

    att_est_scope.udp_send((uint8_t*)(_att_est_debug.data),sizeof(_att_est_debug.data));


    usleep(10000);


  }
}
void * thread_pos_est_scope(void * ptr)
{
  char *addr = (char*)ptr;    
  pos_est_scope.init(addr,3336);
  scope_data_typedef _pos_est_debug;
  while(1)
  {
    pos_est_scope_msg.read(&_pos_est_debug);
    pos_est_scope.udp_send((uint8_t*)(_pos_est_debug.data),sizeof(_pos_est_debug.data));

    usleep(10000);


  }
}

void start_scope(const char *addr)
{ 
  bool ret = create_thread("system_scope", thread_system_scope, (void*)addr);
  ret = create_thread("controller_scope", thread_controller_scope, (void*)addr);
  ret = create_thread("att_est_scope", thread_att_est_scope, (void*)addr);
  ret = create_thread("pos_est_scope", thread_pos_est_scope, (void*)addr);

}

