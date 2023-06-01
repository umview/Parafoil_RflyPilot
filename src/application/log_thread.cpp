#include "log_thread.h"
class binlog_typedef system_log;
class binlog_typedef controller_log;
class binlog_typedef att_est_log;
class binlog_typedef pos_est_log;
void * thread_log(void * dir)
{
    core_bind(LOG_CORE);
    timespec thread_log_sleep;
    thread_log_sleep.tv_sec = 0;
    thread_log_sleep.tv_nsec = 10*1000*1000;//2ms	
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);

	int i = 0;
    scope_data_typedef  _system_debug;
    scope_data_typedef _controller_debug;
    scope_data_typedef _pos_est_debug;
    scope_data_typedef _att_est_debug;
    char system_log_buf[100] = {0};
    char controller_log_buf[100] = {0};
    char att_est_log_buf[100] = {0};
    char pos_est_log_buf[100] = {0};
    char syspath[100] = {0};


    sprintf(syspath, "%s/log-%s", (char*)dir, time_buf);        

    struct stat st = {0};  
    if (stat(syspath, &st) == -1) {
        mkdir(syspath, 0777);
    }

    sprintf(system_log_buf, "%s/log-%s/system_log.bin", (char*)dir, time_buf);        
    sprintf(controller_log_buf, "%s/log-%s/controller_log.bin", (char*)dir, time_buf);        
    sprintf(att_est_log_buf, "%s/log-%s/att_est_log.bin", (char*)dir, time_buf);        
    sprintf(pos_est_log_buf, "%s/log-%s/pos_est_log.bin", (char*)dir, time_buf); 


	while(1)
	{

        //printf("logging\n");

        if(config.sys_log_en)
        {
            system_scope_msg.read(&_system_debug);  
            system_log.binlog_write(system_log_buf,(uint8_t *)&_system_debug,sizeof(scope_data_typedef));
        }
        if(config.ctrl_log_en)
        {
            controller_scope_msg.read(&_controller_debug);
            controller_log.binlog_write(controller_log_buf,(uint8_t *)&_controller_debug,sizeof(scope_data_typedef)); 
        }


        if(config.est_log_en)
        {
            pos_est_scope_msg.read(&_pos_est_debug);
            att_est_scope_msg.read(&_att_est_debug);
            att_est_log.binlog_write(att_est_log_buf,(uint8_t *)&_att_est_debug,sizeof(scope_data_typedef));
            pos_est_log.binlog_write(pos_est_log_buf,(uint8_t *)&_pos_est_debug,sizeof(scope_data_typedef));
        }

	   //binlog_write((char*)dir,binlog_data.buff,sizeof(struct debug_log_typedef));
       nanosleep(&thread_log_sleep,NULL);

	}

}
void start_log(const char * dir)
{
  rflypilot_config_typedef config;
  rflypilot_config_msg.read(&config);    
  bool ret = create_thread("log", thread_log, (void*)config.log_dir);
}

