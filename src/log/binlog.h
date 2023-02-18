#ifndef _BINLOG_
#define _BINLOG_
#include "include.h"
#include <fcntl.h>
void binlog_demo(void);

class binlog_typedef
{
public:
	int fp;
	bool initd;
	void binlog_write(const char *file, uint8_t *buff,uint32_t len);

};


// void binlog_write(const char *file, uint8_t *buff,uint32_t len);
// //void log_data_prepare(void);
// struct debug_log_typedef
// {
// 	float data[80];
// };
// union log_union_typedef
// {
// 	struct debug_log_typedef s;
// 	uint8_t buff[sizeof(struct debug_log_typedef)];
// };
// extern union log_union_typedef binlog_data;

#endif