#include "systime.h"
struct timeval tv;
struct timezone tz;
uint64_t time_in_usec = 0;
uint64_t time_in_usec_last = 0;
//#define fw_variant_ msp::FirmwareVariant::BAFL

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
 
void get_format_time_string(time_t time, const char* format, char *buf)
{
    if (buf == NULL) {
        return;
    }
    struct tm mytm={0};
    struct tm* p_tm = localtime_r(&time,&mytm);
    if (p_tm == NULL) {
        return;
    }
    sprintf(buf, format, p_tm->tm_year + 1900, p_tm->tm_mon + 1, p_tm->tm_mday, p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);
}
 
void get_compact_time_string(time_t time, char* buf) {
    get_format_time_string(time, "%04d-%02d-%02d-%02d-%02d-%02d", buf);
}