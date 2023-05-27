#ifndef _ULOG_THREAD_
#define _ULOG_THREAD_

#include "include.h"
/*
* https://docs.px4.io/main/en/dev_log/ulog_file_format.html
*/
// #ifdef __cplusplus
// extern "C"
// {
// #endif

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
  char format[255];//header.msg_size
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

// #ifdef __cplusplus
// }
// #endif

void * thread_ulog(void * dir);
void start_ulog(const char *dir);

void write_formats(void);
void write_add_logged_msg(void);

#endif