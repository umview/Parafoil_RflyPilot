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
    cf_output_typedef _cf_msg;
    gps_msg_typedef _gps_msg;
    gyro_typedef _gyro_msg;
    accel_typedef _accel_msg;
    

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

    /* Write flag bit */
    write_flag_bits(system_log_buf);

    /* Write Definitions */

        /* lpe message */
        const char lpe_format[] = "Local Position Estimator:uint64_t timestamp;double[3] pos_ned;double[3] vel_ned;double[3] pos_accel_body;double[3] accel_bias;";
        write_format(system_log_buf, lpe_format);

        /* Q Estimator (cf) message */
        const char cf_format[] = "Q Estimator:uint64_t timestamp;double[4] quat;double roll;double pitch;double yaw;";
        write_format(system_log_buf, cf_format);

        /* gps message */
        const char gps_format[] = "gps:bool updated;bool ned_origin_valid;bool gps_is_good;uint8_t[5] _padding0;uint64_t timestamp;double lon;double lat;double lon_origin;double lat_origin;float yaw_offset;float height;float[3] vel_ned;float[3] pos_ned;float hacc;float vacc;float sacc;float heading;float headacc;uint8_t numSV;uint8_t fixType;uint8_t[2] _padding0";//uint8_t[2] _padding0;
        write_format(system_log_buf, gps_format);

        /* gyro message */
        const char gyro_format[] = "gyro:uint64_t timestamp;float[3] gyro;uint8_t[4] _padding0;";//uint8_t[4] _padding0;
        write_format(system_log_buf, gyro_format);

        /* gyro message */
        const char accel_format[] = "accel:uint64_t timestamp;float[3] accel;uint8_t[4] _padding0;";//uint8_t[4] _padding0;
        write_format(system_log_buf, accel_format);

    /* Write Subscription Message */

        /* lpe logged msg */
        const char lpe_msg_name[]="Local Position Estimator";
        write_add_logged_msg(system_log_buf, lpe_msg_name, 0U, 0U);

        /* Q Estimator(cf) logged msg */
        const char cf_msg_name[]="Q Estimator";
        write_add_logged_msg(system_log_buf, cf_msg_name, 1U, 0U);

        /* gps logged msg */
        const char gps_msg_name[]="gps";
        write_add_logged_msg(system_log_buf, gps_msg_name, 2U, 0U);

        /* gyro logged msg */
        const char gyro_msg_name[]="gyro";
        write_add_logged_msg(system_log_buf, gyro_msg_name, 3U, 0U);

        /* accel logged msg */
        const char accel_msg_name[]="accel";
        write_add_logged_msg(system_log_buf, accel_msg_name, 4U, 0U);

	while(1)
	{

        if(config.sys_log_en)
        {
            /* lpe */
            if(lpe_output_msg.read(&_lpe_msg))
            {
                size_t msg_data_len = sizeof(_lpe_msg);
                write_msg(system_log_buf, (uint8_t *)&_lpe_msg, msg_data_len, 0U);
            }

            /* cf */
            if(cf_output_msg.read(&_cf_msg)){
                size_t cf_msg_data_len = sizeof(_cf_msg);
                write_msg(system_log_buf, (uint8_t *)&_cf_msg, cf_msg_data_len, 1U);
            }

            /* gps */
            if(gps_msg.read(&_gps_msg)){
                size_t gps_msg_data_len = sizeof(_gps_msg)-2;
                write_msg(system_log_buf, (uint8_t *)&_gps_msg, gps_msg_data_len, 2U);
            }

            if(gyro_msg.read(&_gyro_msg)){
                size_t gyro_msg_data_len = sizeof(_gyro_msg)-4;
                write_msg(system_log_buf, (uint8_t *)&_gyro_msg, gyro_msg_data_len, 3U);
            }

            if(accel_msg.read(&_accel_msg)){
                size_t accel_msg_data_len = sizeof(_accel_msg)-4;//subtract the last padding length
                write_msg(system_log_buf, (uint8_t *)&_accel_msg, accel_msg_data_len, 4U);
            }
        }

       nanosleep(&thread_log_sleep, NULL);

	}

}
void write_flag_bits(const char *filename)
{   
    ulog_message_flag_bits_s msg = {
        .header = {
            .msg_size = 40U,
            .msg_type = 'B'
        },
        .compat_flags = {0U,},
        .incompat_flags = {0U,},
        .appended_offsets = {0U,}
    };
    size_t msg_size = sizeof(msg);
    system_ulog.binlog_write(filename, (uint8_t *)&msg, msg_size);
}

void write_msg(const char *filename, uint8_t *logged_data, size_t logged_data_len, uint16_t msg_id)
{
    /* create msg data and cal the size of msg_data */
    struct message_data_s msg;
    size_t msg_data_size = sizeof(msg) - sizeof(msg.data) + logged_data_len;

    /* set msg data content */
    msg.header.msg_size = msg_data_size - sizeof(msg.header);
    msg.header.msg_type = 'D';
    msg.msg_id = msg_id;//unique ID
    memcpy(msg.data, logged_data, logged_data_len);

    /* write to file */
    system_ulog.binlog_write(filename, (uint8_t *)&msg, msg_data_size);
}

// void write_add_logged_msgs()
// {
//     const char filename[]= "ulog";

//     /* lpe logged msg */
//     const char lpe_msg_name[]="lpe";
//     write_add_logged_msg(filename, lpe_msg_name, 0, 0);

//     /* Q Estimator(cf) logged msg */
//     const char cf_msg_name[]="Q Estimator";
//     write_add_logged_msg(filename, cf_msg_name, 1, 0);
// }

void write_add_logged_msg(const char *filename, const char *msg_name, uint16_t msg_id, uint8_t multi_id)
{
    /* msg name len */
    uint16_t msg_name_len = strlen(msg_name);

    /* msg size */
    struct message_add_logged_s msg;
    size_t msg_size = sizeof(msg) - sizeof(msg.message_name) + msg_name_len;

    /* construct msg */
    msg.header.msg_size = msg_size - sizeof(msg.header);
    msg.header.msg_type = 'A';
    msg.msg_id = msg_id;
    msg.multi_id = multi_id;
    memcpy(msg.message_name, msg_name, msg_name_len);

    /* write to file */
    system_ulog.binlog_write(filename, (uint8_t *)&msg, msg_size);
}

// void write_formats()
// {
//     const char filename[]= "ulog";

//     /* lpe message */
//     const char lpe_format[] = "lpe:uint64_t timestamp;double[3] pos_ned;double[3] vel_ned;double[3] pos_accel_body;double[3] accel_bias;";
//     write_format(filename, lpe_format);

//     /* Q Estimator (cf) message */
//     const char cf_format[] = "Q Estimator:uint64_t timestamp;double[4] quat;double roll;double pitch;double yaw;";
//     write_format(filename, cf_format);
// }

void write_format(const char *filename ,const char *msg_format)
{
    /* format length */
    uint16_t msg_format_len = strlen(msg_format);

    /* msg size */
    struct message_format_s msg;
    size_t msg_size = sizeof(msg) - sizeof(msg.format) + msg_format_len;

    /* construct msg */
    msg.header.msg_size = msg_size - sizeof(msg.header);
    msg.header.msg_type = 'F';
    memcpy(msg.format, msg_format, msg_format_len);
    
    system_ulog.binlog_write(filename, (uint8_t *)&msg, msg_size);
}

void start_ulog(const char * dir)
{
  rflypilot_config_typedef config;
  rflypilot_config_msg.read(&config);    
  bool ret = create_thread("ulog", thread_ulog, (void*)config.log_dir);
}

