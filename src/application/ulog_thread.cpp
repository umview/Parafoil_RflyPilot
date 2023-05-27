#include "ulog_thread.h"
class binlog_typedef system_ulog;

void * thread_ulog(void * dir)
{
    timespec thread_log_sleep;
    thread_log_sleep.tv_sec = 0;
    thread_log_sleep.tv_nsec = 10*1000*1000;//2ms	
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);

	int i = 0;
    scope_data_typedef  _system_debug;
    lpe_output_typedef _lpe_msg;
    

    char time_buf[50]={0};
    char system_log_buf[100] = {0};
    char controller_log_buf[100] = {0};
    char att_est_log_buf[100] = {0};
    char pos_est_log_buf[100] = {0};
    char syspath[100] = {0};
    get_compact_time_string(time(NULL), time_buf);   

    sprintf(syspath, "%s/log-%s", (char*)dir, time_buf);        

    struct stat st = {0};  
    if (stat(syspath, &st) == -1) {
        mkdir(syspath, 0777);
    }

    sprintf(system_log_buf, "%s/log-%s/system_ulog.ulg", (char*)dir, time_buf);

    /* Write header */
    struct ulog_header_s header = {
        .file_magic = {0x55U, 0x4cU, 0x6fU, 0x67U, 0x01U, 0x12U, 0x35U},
        .version = 1U,
        .timestamp = get_time_now()
    };
    system_ulog.binlog_write(system_log_buf, (uint8_t *)&header, sizeof(header));

    /* Write Definitions */
    write_formats();

    /* Write Subscription Message */
    write_add_logged_msg();

	while(1)
	{

        if(config.sys_log_en)
        {
            /* read the data would be logged */
            lpe_output_msg.read(&_lpe_msg);
            size_t msg_data_len = sizeof(_lpe_msg);

            /* create msg data and cal the size of msg_data */
            struct message_data_s msg_data;
            size_t msg_data_size = sizeof(msg_data) - sizeof(msg_data.data) + msg_data_len;

            /* set msg data content */
            msg_data.header.msg_size = msg_data_size - sizeof(msg_data.header);
            msg_data.header.msg_type = 'D';
            msg_data.msg_id = 0U;
            memcpy(msg_data.data, &_lpe_msg, msg_data_len);

            /* write to file */
            system_ulog.binlog_write(system_log_buf, (uint8_t *)&msg_data, msg_data_size);
        }

       nanosleep(&thread_log_sleep, NULL);

	}

}
void write_add_logged_msg()
{
    const char lpe_msg_name[]="lpe";
    uint16_t lpe_msg_name_len = strlen(lpe_msg_name);

    struct message_add_logged_s msg;
    size_t msg_size = sizeof(msg) - sizeof(msg.message_name) + lpe_msg_name_len;

    msg.header.msg_size = msg_size - sizeof(msg.header);
    msg.header.msg_type = 'A';
    msg.msg_id = 0U;
    msg.multi_id = 0U;
    memcpy(msg.message_name, lpe_msg_name, lpe_msg_name_len);

    const char filename[]= "ulog";
    system_ulog.binlog_write(filename, (uint8_t *)&msg, msg_size);

}

void write_formats()
{
    /* lpe message */
    const char lpe_format[] = "lpe:uint64_t timestamp;double[3] pos_ned;double[3] vel_ned;double[3] pos_accel_body;double[3] accel_bias;";
    uint16_t lpe_format_len = strlen(lpe_format);
    struct message_format_s lpe_format_s;
    lpe_format_s.header.msg_size = lpe_format_len;
    lpe_format_s.header.msg_type = 'F';
    memcpy(lpe_format_s.format, lpe_format, lpe_format_len);
    
    const char filename[]= "ulog";
    system_ulog.binlog_write(filename, (uint8_t *)&lpe_format_s, sizeof(lpe_format_s.header) + lpe_format_len);
}

void start_ulog(const char * dir)
{
  rflypilot_config_typedef config;
  rflypilot_config_msg.read(&config);    
  bool ret = create_thread("ulog", thread_ulog, (void*)config.log_dir);
}

