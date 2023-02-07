#ifndef _SYSTIME_H_
#define _SYSTIME_H_
#include "include.h"
uint64_t get_time_now(void);
extern uint64_t get_time_now(void);
void get_format_time_string(time_t time, const char* format, char *buf);
void get_compact_time_string(time_t time, char* buf);
#endif