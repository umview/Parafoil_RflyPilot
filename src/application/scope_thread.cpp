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
    gyro_typedef _gyro;
    accel_typedef _accel;
    scope_data_typedef  _system_debug_data;
  while(1)
  {
    cf_output_msg.read(&_att);
    lpe_output_msg.read(&_lpe);
    gps_msg.read(&_gps);
    mag_msg.read(&_mag);
    actuator_output_msg.read(&_actuator_output);
    gyro_msg.read(&_gyro);
    accel_msg.read(&_accel);
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



    system_scope_msg.publish(&_system_debug_data);  
	system_scope.udp_send((uint8_t*)(_system_debug_data.data),sizeof(_system_debug_data.data));
    



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

