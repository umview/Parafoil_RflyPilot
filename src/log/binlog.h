#ifndef _BINLOG_
#define _BINLOG_
#include "config.h"
#include <fcntl.h>
void binlog_demo(void);
void binlog_write(void);
void log_data_prepare(void);
struct debug_log_typedef
{
	float data[80];
};
union log_union_typedef
{
	struct debug_log_typedef s;
	uint8_t buff[sizeof(struct debug_log_typedef)];
};
extern union log_union_typedef binlog_data;

#endif