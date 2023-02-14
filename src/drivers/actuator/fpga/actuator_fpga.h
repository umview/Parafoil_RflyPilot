#ifndef _ACTUATOR_FPGA_
#define _ACTUATOR_FPGA_
#include "include.h"
#define ACTUATOR_FPGA_DEVICE_BASE_PATH "/dev/i2c-1"
#define ACTUATOR_FPGA_DEVICE_ADDRESS 0x2E
void start_actuator_fpga(void);
enum actuator_mode_typdef
{
	UNDEFINED = 0,
	PWM400,
	ONESHOT125,
};
#define ACTUATOR_FPGA_N 8
class actuator_fpga_typedef
{
public:
	uint8_t buffer[3*2*ACTUATOR_FPGA_N+1];
	actuator_fpga_typedef(void);
	bool setPWM(int _num_chn, actuator_mode_typdef *mode, float *duty);
	void demo(void);
	void init(void);
	//bool actuator_fpga_init(int _num_chn, int *freq_hz, float *duty_0);
};
#endif