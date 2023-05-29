#include "ulog_thread.h"
class binlog_typedef system_ulog;

void * thread_ulog(void * dir)
{
    timespec thread_log_sleep;
    thread_log_sleep.tv_sec = 0;
    thread_log_sleep.tv_nsec = 20*1000*1000;//2ms	
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);

    /* Declare msg */
    // scope_data_typedef  _system_debug;
    lpe_output_typedef _lpe_msg;
    lpe_output_typedef _lpeLowPass_msg;
    cf_output_typedef _cf_msg;

    gps_msg_typedef _gps_msg;    
    gyro_typedef _gyro_msg;
    accel_typedef _accel_msg;
    baro_typedef _baro_msg;
    mag_typedef _mag_msg;

    actuator_output_typedef _actuator_output_msg;
    actuator_output_typedef _aux_actuator_output_msg;

    sbus_packet_t _rc_input_msg;
    
    /* Set ulog path */
    char time_buf[50]={0};
    char syspath[100] = {0};
    get_compact_time_string(time(NULL), time_buf);   
    sprintf(syspath, "%s/log-%s", (char*)dir, time_buf);        
    struct stat st = {0};  
    if (stat(syspath, &st) == -1) {
        mkdir(syspath, 0777);
    }

    /* Define ulog file name */
    char ulog_name[100] = {0};
    sprintf(ulog_name, "%s/log-%s/ulog-%s.ulg", (char*)dir, time_buf, time_buf);

    /* Write header */
    struct ulog_header_s header = {
        .file_magic = {0x55U, 0x4cU, 0x6fU, 0x67U, 0x01U, 0x12U, 0x35U},
        .version = 1U,
        .timestamp = get_time_now()
    };
    system_ulog.binlog_write(ulog_name, (uint8_t *)&header, sizeof(header));

    /* Write flag bit */
    write_flag_bits(ulog_name);

    /* Write Definitions */

        /* lpe message */
        const char lpe_format[] = "Local Position Estimator:uint64_t timestamp;double[3] pos_ned;double[3] vel_ned;double[3] pos_accel_body;double[3] accel_bias;";
        write_format(ulog_name, lpe_format);

        /* lpeLowPass message */
        const char lpeLowPass_format[] = "Local Position Estimator LowPass:uint64_t timestamp;double[3] pos_ned;double[3] vel_ned;double[3] pos_accel_body;double[3] accel_bias;";
        write_format(ulog_name, lpeLowPass_format);

        /* Q Estimator (cf) message */
        const char cf_format[] = "Q Estimator:uint64_t timestamp;double[4] quat;double roll;double pitch;double yaw;";
        write_format(ulog_name, cf_format);

        /* gps message */
        const char gps_format[] = "gps:bool updated;bool ned_origin_valid;bool gps_is_good;uint8_t[5] _padding0;uint64_t timestamp;double lon;double lat;double lon_origin;double lat_origin;float yaw_offset;float height;float[3] vel_ned;float[3] pos_ned;float hacc;float vacc;float sacc;float heading;float headacc;uint8_t numSV;uint8_t fixType;uint8_t[2] _padding0";//uint8_t[2] _padding0;
        write_format(ulog_name, gps_format);

        /* gyro message */
        const char gyro_format[] = "sensor gyro:uint64_t timestamp;float[3] gyro;uint8_t[4] _padding0;";//uint8_t[4] _padding0;
        write_format(ulog_name, gyro_format);

        /* accel message */
        const char accel_format[] = "sensor accel:uint64_t timestamp;float[3] accel;uint8_t[4] _padding0;";//uint8_t[4] _padding0;
        write_format(ulog_name, accel_format);

        /* mag message */
        const char mag_format[] = "sensor mag:uint64_t timestamp;float[3] mag;uint8_t[4] _padding0;";//uint8_t[4] _padding0;
        write_format(ulog_name, mag_format);

        /* baro message */
        const char baro_format[] = "sensor baro:uint64_t timestamp;double mag;double temperature;";
        write_format(ulog_name, baro_format);

        /* actuator message format */
        const char actuator_format[] = "actuator rotor:uint64_t timestamp;uint16_t[8] actuator_output;";
        write_format(ulog_name, actuator_format);

        /* Servo actuator message format */
        const char servo_actuator_format[] = "actuator servo:uint64_t timestamp;uint16_t[8] actuator_output;";
        write_format(ulog_name, servo_actuator_format);

        /* rc input message format */
        const char rc_format[] = "rc input:uint64_t timestamp;uint16_t[16] channels;bool ch17;bool ch18;bool failsafe;bool frameLost;uint8_t[4] _padding0";// last padding = 4
        write_format(ulog_name, rc_format);

    /* Write Subscription Message */

        /* lpe logged msg */
        const char lpe_msg_name[]="Local Position Estimator";
        write_add_logged_msg(ulog_name, lpe_msg_name, 0U, 0U);

        /* Q Estimator(cf) logged msg */
        const char cf_msg_name[]="Q Estimator";
        write_add_logged_msg(ulog_name, cf_msg_name, 1U, 0U);

        /* gps logged msg */
        const char gps_msg_name[]="gps";
        write_add_logged_msg(ulog_name, gps_msg_name, 2U, 0U);

        /* gyro logged msg */
        const char gyro_msg_name[]="sensor gyro";
        write_add_logged_msg(ulog_name, gyro_msg_name, 3U, 0U);

        /* accel logged msg */
        const char accel_msg_name[]="sensor accel";
        write_add_logged_msg(ulog_name, accel_msg_name, 4U, 0U);

        /* mag logged msg */
        const char mag_msg_name[]="sensor mag";
        write_add_logged_msg(ulog_name, mag_msg_name, 5U, 0U);

        /* baro logged msg */
        const char baro_msg_name[]="sensor baro";
        write_add_logged_msg(ulog_name, baro_msg_name, 6U, 0U);

        /* lpeLowPass logged msg */
        const char lpeLowPass_msg_name[]="Local Position Estimator LowPass";
        write_add_logged_msg(ulog_name, lpeLowPass_msg_name, 7U, 0U);

        /* actuator logged msg */
        const char actuator_msg_name[]="actuator rotor";
        write_add_logged_msg(ulog_name, actuator_msg_name, 8U, 0U);

        /* servo logged msg */
        const char servo_msg_name[]="actuator servo";
        write_add_logged_msg(ulog_name, servo_msg_name, 9U, 0U);

        /* lpeLowPass logged msg */
        const char rc_msg_name[]="rc input";
        write_add_logged_msg(ulog_name, rc_msg_name, 10U, 0U);

	while(1)
	{

        if(config.sys_log_en)
        {
            /* lpe */
            if(lpe_output_msg.read(&_lpe_msg))
            {
                size_t msg_data_len = sizeof(_lpe_msg);
                write_msg(ulog_name, (uint8_t *)&_lpe_msg, msg_data_len, 0U);
            }

            /* cf */
            if(cf_output_msg.read(&_cf_msg)){
                size_t cf_msg_data_len = sizeof(_cf_msg);
                write_msg(ulog_name, (uint8_t *)&_cf_msg, cf_msg_data_len, 1U);
            }

            /* gps */
            if(gps_msg.read(&_gps_msg)){
                size_t gps_msg_data_len = sizeof(_gps_msg)-2;//subtract the last padding length
                write_msg(ulog_name, (uint8_t *)&_gps_msg, gps_msg_data_len, 2U);
            }

            /* gyro */
            if(gyro_msg.read(&_gyro_msg)){
                size_t gyro_msg_data_len = sizeof(_gyro_msg)-4;//subtract the last padding length
                write_msg(ulog_name, (uint8_t *)&_gyro_msg, gyro_msg_data_len, 3U);
            }

            /* accel */
            if(accel_msg.read(&_accel_msg)){
                size_t accel_msg_data_len = sizeof(_accel_msg)-4;//subtract the last padding length
                write_msg(ulog_name, (uint8_t *)&_accel_msg, accel_msg_data_len, 4U);
            }

            if(mag_msg.read(&_mag_msg)){
                size_t mag_msg_data_len = sizeof(_mag_msg)-4;//subtract the last padding length
                write_msg(ulog_name, (uint8_t *)&_mag_msg, mag_msg_data_len, 5U);
            }

            if(baro_msg.read(&_baro_msg)){
                size_t baro_msg_data_len = sizeof(_baro_msg);
                write_msg(ulog_name, (uint8_t *)&_baro_msg, baro_msg_data_len, 6U);
            }

            if(lpeLowPass_output_msg.read(&_lpeLowPass_msg))
            {
                size_t lpeLowPass_data_len = sizeof(_lpeLowPass_msg);
                write_msg(ulog_name, (uint8_t *)&_lpeLowPass_msg, lpeLowPass_data_len, 7U);
            }

            if(actuator_output_msg.read(&_actuator_output_msg)){
                size_t actuator_msg_data_len = sizeof(_actuator_output_msg);
                write_msg(ulog_name, (uint8_t *)&_actuator_output_msg, actuator_msg_data_len, 8U);
            }

            if(aux_actuator_output_msg.read(&_aux_actuator_output_msg)){
                size_t aux_actuator_msg_data_len = sizeof(_aux_actuator_output_msg);
                write_msg(ulog_name, (uint8_t *)&_aux_actuator_output_msg, aux_actuator_msg_data_len, 9U);
            }

            if(rc_input_msg.read(&_rc_input_msg)){
                size_t rc_input_msg_data_len = sizeof(_rc_input_msg)-4;
                write_msg(ulog_name, (uint8_t *)&_rc_input_msg, rc_input_msg_data_len, 10U);
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

