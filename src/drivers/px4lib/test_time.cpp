#ifndef _TEST_H_
#define _TEST_H_

#include "test_time.h"
#ifndef TESTMODE
#else


struct timeval tv;
struct timezone tz;
uint64_t time_in_usec = 0;
uint64_t time_in_usec_last = 0;
static uint64_t _time_now(void)
{
    gettimeofday(&tv, &tz);
    time_in_usec_last = time_in_usec;
    time_in_usec = tv.tv_sec * 1e6 + tv.tv_usec;
    return time_in_usec;
}
uint64_t get_time_now(void)
{
    static uint64_t start_time = _time_now();
    return _time_now() - start_time;
}
#endif

#endif