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

    scope_data_typedef  _system_debug_data;
  while(1)
  {


    system_scope_msg.read(&_system_debug_data);  
    rflysim3d_output_msg.read(&_rflysim3d_output_msg);
	  system_scope.udp_send((uint8_t*)(_system_debug_data.data),sizeof(_system_debug_data.data));  
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
  bool ret;
  rflypilot_config_typedef config;
  rflypilot_config_msg.read(&config);
  if(config.sys_scope_en)
  {
    ret = create_thread("system_scope", thread_system_scope, (void*)config.scope_ip);
  }

  if(config.ctrl_scope_en)
  {
    ret = create_thread("controller_scope", thread_controller_scope, (void*)config.scope_ip);
  }

  if(config.est_scope_en)
  {
    ret = create_thread("att_est_scope", thread_att_est_scope, (void*)config.scope_ip);
    ret = create_thread("pos_est_scope", thread_pos_est_scope, (void*)config.scope_ip);
  }


}

