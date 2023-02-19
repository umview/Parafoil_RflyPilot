#ifndef _UTILITY_SYSTEM_
#define _UTILITY_SYSTEM_
#include "include.h"

bool core_bind(int cpu_index);
void set_thread_policy(pthread_attr_t *attr, int policy,int priority);
#define POLL_TIME_US 10*1000
class scheduler_typedef
{
public:
	int base_timer_rate;
	bool controller_flag;
	bool imu_flag;
	bool att_est_flag;
	bool lpe_flag;
	bool mag_flag;
	int lpe_flag_cnt;
	int att_est_flag_cnt;
	int mag_flag_cnt;
	int controller_flag_cnt;
	int imu_flag_cnt;
	void start_system_timer(int base_timer);
	void scheduler_timer_fcn(void);
	scheduler_typedef(void);
};
extern class scheduler_typedef scheduler;

class adaptive_delay_typedef
{
public:
    timespec _sleep;
    bool initd; 
    double delay_us_feedback;
	double integral;
	double kp_output;
	long delay_ns_output; 
	double ff;
	double err;
	float kp;
	float ki;
	uint64_t offset_us;
	uint64_t t0_us;
	uint64_t t1_us;
	uint64_t delay_us_feedback_last;
	float lpf_k;
	bool locked;
	int lock_cnt;
	// offset_us 为程序本身所消耗的时间 单位us
	adaptive_delay_typedef(float _kp, float _ki, uint64_t _offset_us);
	void delay_us(uint64_t us);
	uint64_t calc_delay_us(uint64_t us);

};
void scheduler_timer(void);

void delay_us_combined(uint64_t _delay_us, bool *timer_flag, class adaptive_delay_typedef *adp_delay);

bool create_thread(const char *name, void* (*start_rtn)(void*),void *thread_arg);
#endif