#ifndef _ULOG_THREAD_
#define _ULOG_THREAD_

#include "include.h"
/*
* https://docs.px4.io/main/en/dev_log/ulog_file_format.html
*/

#pragma pack(push, 1)
struct ulog_header_s {
    uint8_t file_magic[7];
    uint8_t version;
    uint64_t timestamp;
};

struct message_header_s {
  uint16_t msg_size;
  uint8_t msg_type;
};

struct message_format_s {
  struct message_header_s header; // msg_type = 'F'
  char format[1500];//header.msg_size
};

struct message_add_logged_s {
  struct message_header_s header; // msg_type = 'A'
  uint8_t multi_id;
  uint16_t msg_id;
  char message_name[255];//header.msg_size-3
};

struct message_data_s {
  struct message_header_s header; // msg_type = 'D'
  uint16_t msg_id;
  uint8_t data[255];//header.msg_size-2
};

struct ulog_message_flag_bits_s {
  struct message_header_s header; // msg_type = 'B'
  uint8_t compat_flags[8];
  uint8_t incompat_flags[8];
  uint64_t appended_offsets[3]; // file offset(s) for appended data if appending bit is set
};


#pragma pack(pop)

struct msg_rate {
  uint64_t timestamp;
  float usr_controller_rate;
  float q_estimator_rate;
  float local_position_estimator_rate;
  float sensor_accel_rate;
  float sensor_gyro_rate;
  float sensor_mag_rate;
  float sensor_baro_rate;
  float sensor_gps_rate;
  float rc_input_rate;
};

// // #pragma pack(push, 8)
// typedef struct struct_test1
// {
//     bool g1;
//     bool g2;
//     bool g3;
//     bool g4;
//     bool g5;
//     bool g6;
//     bool g7;
//     bool g8;
//     // bool g9;
//     // bool g10;
//     // bool g11;
//     // bool g12;
//     double b;
//     bool c;
// }ST1;
// // #pragma pack(pop)
// size_t size = sizeof(ST1);

void * thread_ulog(void * dir);
void start_ulog(const char *dir);

// void write_formats(void);
void write_format(const char *filename ,const char *msg_format);

// void write_add_logged_msgs(void);
void write_add_logged_msg(const char *filename, const char *msg_name, uint16_t msg_id, uint8_t multi_id);

void write_msg(const char *filename, uint8_t *logged_data, size_t logged_data_len, uint16_t msg_id);

void write_flag_bits(const char *filename);

void cp_file (char *path_from, char *path_to);

int find_log_index(void);

#endif