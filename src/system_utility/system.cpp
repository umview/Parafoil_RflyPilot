#include "system.h"
bool core_bind(int cpu_index)
{
	cpu_set_t mask;
	if(cpu_index >= 0 && cpu_index <= 3)
	{
	     CPU_ZERO(&mask);
	     CPU_SET(cpu_index,&mask);
	     if (sched_setaffinity(0,sizeof(mask),&mask)==-1)
	             printf("affinity set fail!");
		return true;
	}else{
		printf("error core index %d\n", cpu_index);
		return false;
	}
}
void set_thread_policy(pthread_attr_t *attr, int policy,int priority)
{
			struct sched_param param;
            // 初始化线程属性
            pthread_attr_init(attr);
            
            // 设置调度策略为：SCHED_FIFO
            pthread_attr_setschedpolicy(attr, policy);
            
            param.sched_priority  = priority;
            pthread_attr_setschedparam(attr, &param);
            
            // 设置线程属性：不要继承 main 线程的调度策略和优先级。
            pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
}

adaptive_delay_typedef::adaptive_delay_typedef(float _kp, float _ki,  uint64_t _offset_us)
{
	kp = _kp;
	ki = _ki;
	offset_us = _offset_us;
	integral = 0;
	lpf_k = 0.2;
	initd = false;
	locked = false;
	lock_cnt = 0;
}
uint64_t adaptive_delay_typedef::calc_delay_us(uint64_t us)
{
	uint64_t deltaT = 0; 
	if(!initd)
	{
		initd = true;
		t0_us = get_time_now();
		deltaT = us;
		delay_us_feedback_last = us;
		return deltaT;
	}
	t1_us = get_time_now();
	deltaT = t1_us - t0_us;
	t0_us = t1_us;
	return deltaT;
}
void adaptive_delay_typedef::delay_us(uint64_t us)
{
	if(us < offset_us)
	{
		printf("ERROR : too short time ! %ld\n", us);
		return;
	}
	delay_us_feedback = (lpf_k * calc_delay_us(us) + (1-lpf_k) * delay_us_feedback_last);
	err = us - delay_us_feedback;
	if(fabsf(err / us) < 0.05 && !locked)
	{
		lock_cnt ++;
		if(lock_cnt > 3000)locked = true;
	}else{
		lock_cnt = 0;
	}
	if(!locked)
	{
		ff = us - offset_us;
		kp_output = 0.01 * err;
		integral += ki * err * us / 1e6;
		delay_ns_output = (long)((kp_output + integral + ff) * 1e3);
		if(delay_ns_output > 100 * 1e3 * ff)delay_ns_output = 100 * 1e3 * ff;
		if(delay_ns_output < 1e-3 * 1e3 * ff)delay_ns_output = 1e-3 * 1e3 * ff;
	}

    _sleep.tv_sec = 0;
    _sleep.tv_nsec = delay_ns_output;
    nanosleep(&_sleep,NULL);
}

class scheduler_typedef scheduler;
void scheduler_timer(void)
{
	scheduler.scheduler_timer_fcn();
}
void  scheduler_typedef::scheduler_timer_fcn(void)
{
	if(mpc_flag_cnt++ == (int)(base_timer_rate/config.mpc_rate))
	{
		mpc_flag = true;
		mpc_flag_cnt = 1;
	}
	if(actuator_flag_cnt++ == (int)(base_timer_rate/config.actuator_rate))
	{
		actuator_flag = true;
		actuator_flag_cnt = 1;
	}
	if(pid_controller_flag_cnt++ == (int)(base_timer_rate/config.pid_controller_rate))
	{
		pid_controller_flag = true;
		pid_controller_flag_cnt = 1;
	}
	if(att_est_flag_cnt++ == (int)(base_timer_rate/config.attitude_est_rate))
	{
		att_est_flag = true;
		att_est_flag_cnt = 1;
	}
	if(lpe_flag_cnt++ == (int)(base_timer_rate/config.lpe_rate))
	{
		lpe_flag = true;
		lpe_flag_cnt = 1;
	}
	if(imu_flag_cnt++ == (int)(base_timer_rate/config.imu_rate))
	{
		imu_flag = true;
		imu_flag_cnt = 1;
	}
}

scheduler_typedef::scheduler_typedef(void)
{
	mpc_flag = true;
	pid_controller_flag = true;
	imu_flag = true;
	actuator_flag = true;
	att_est_flag = true;
	lpe_flag = true;
	
}
void scheduler_typedef::start_system_timer(int _base_timer_rate)
{
    int ret;
    timer_t timer;
    struct sigevent evp;
    struct timespec spec;
    struct itimerspec time_value;
    base_timer_rate = _base_timer_rate;
    evp.sigev_value.sival_ptr = &timer;
    /*定时器到期时，会产生一个信号*/
    evp.sigev_notify = SIGEV_SIGNAL; 
    evp.sigev_signo = SIGUSR1;
    signal(SIGUSR1, (__sighandler_t)scheduler_timer);

    /*时钟源选CLOCK_MONOTONIC主要是考虑到系统的实时时钟可能会在
    程序运行过程中更改，所以存在一定的不确定性，而CLOCK_MONOTONIC
    则不会，较为稳定*/
    //ret = timer_create(CLOCK_MONOTONIC, &evp, &timer);
    if( ret )
        perror("timer_create");

    time_value.it_interval.tv_sec = 0;      /*每秒触发一次*/
    time_value.it_interval.tv_nsec = (uint32_t)(1000000000.f/base_timer_rate);//10khz timer base
    //clock_gettime(CLOCK_MONOTONIC, &spec);         
    time_value.it_value.tv_sec = spec.tv_sec + 0;      /*5秒后启动*/
    time_value.it_value.tv_nsec = spec.tv_nsec + 0;

    //ret = timer_settime(timer, CLOCK_MONOTONIC, &time_value, NULL);
    if( ret )
            perror("timer_settime");

}