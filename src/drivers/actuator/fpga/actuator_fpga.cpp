#include "actuator_fpga.h"

class i2c i2c_actuator_fpga(ACTUATOR_FPGA_DEVICE_ADDRESS);

class actuator_fpga_typedef actuator_fpga;
// bool actuator_fpga_typedef::actuator_fpga_init(int _num_chn, int *freq_hz, float *duty_0)
// {

// }
actuator_fpga_typedef::actuator_fpga_typedef(void)
{
	memset(buffer, 0u, sizeof(buffer));
}
void actuator_fpga_typedef::init(void)
{
	int ret = 0;
	ret = i2c_actuator_fpga.init(ACTUATOR_FPGA_DEVICE_BASE_PATH);
    if (OK != ret) {
        printf("i2c_actuator_fpga.init failed (%i)", ret);
	}
}
bool actuator_fpga_typedef::setPWM(int _num_chn, actuator_mode_typdef *mode, float *duty)
{
	uint32_t data_period = 0; 
	uint32_t data_duty = 0;
	//uint8_t data[6] = {0,0,0,0,0,0}; 
	buffer[0] = 0x00; //reg address
	if(_num_chn > ACTUATOR_FPGA_N)
	{
		printf("ACTUATOR_FPGA CHANNEL NUMMBER ERROR !\n");
		_num_chn = ACTUATOR_FPGA_N;
	}
	for(int i = 0; i < _num_chn; i++)
	{
		switch(mode[i])
		{
			case PWM400:
				data_period = (uint32_t)(48000000.f / 400);
				data_duty = (uint32_t)(duty[i] / 2500 * (48000000.f / 400));
			break;

			case ONESHOT125:
				data_period = (uint32_t)(48000000.f / 4000);
				data_duty = (uint32_t)(duty[i] / 2500 * (48000000.f / 4000));
			break;

			default:
				data_period = 0;
				data_duty = 0;			
				printf("actuator fpga : undefined mode %d\n", (int)mode[i]);
			break;
		}
		//data_period = (uint32_t)(48000000 / freq_hz[i]);
		//data_duty = (uint32_t)(duty[i] / (1000000 / freq_hz[i]) * (48000000 / freq_hz[i]));
		buffer[i * 6 + 1] = (data_period << 24) >> 24;
		buffer[i * 6 + 2] = (data_period << 16) >> 24;
		buffer[i * 6 + 3] = (data_period << 8) >> 24;
		buffer[i * 6 + 4] = (data_duty << 24) >> 24;
		buffer[i * 6 + 5] = (data_duty << 16) >> 24;
		buffer[i * 6 + 6] = (data_duty << 8) >> 24;		
	}
	int ret = i2c_actuator_fpga.transfer(buffer, _num_chn * 6 + 1, nullptr, 0);
	if (OK != ret) {
		printf("set_pwm: i2c::transfer returned %d\n\r", ret);
	}
	return ret;
}
void actuator_fpga_typedef::demo(void)
{
	uint8_t buff[] = {0x00,0xE0, 0x2E, 0x00, 0x70, 0x17, 0x00};
	uint8_t read_buff[6] = {0};
	int ret = i2c_actuator_fpga.transfer(buff, sizeof(buff), nullptr, 0);
	usleep(50000);
	uint8_t reg[] = {0x00};
	ret = i2c_actuator_fpga.transfer(reg, 1, read_buff, sizeof(read_buff));
	printf("read: %x %x %x %x %x %x\n",
				read_buff[0],
				read_buff[1],
				read_buff[2],
				read_buff[3],
				read_buff[4],
				read_buff[5]);
	if (OK != ret) {
		printf("set_pwm: i2c::transfer returned %d\n\r", ret);
	}
}



void *thread_actuator_fpga(void *ptr)
{
	int ret = i2c_actuator_fpga.init(ACTUATOR_FPGA_DEVICE_BASE_PATH);
    if (OK != ret) {
        printf("i2c_actuator_fpga.init failed (%i)", ret);
        return NULL;
	}
	actuator_mode_typdef _mode[] = {ONESHOT125,ONESHOT125,ONESHOT125,ONESHOT125,ONESHOT125,ONESHOT125,ONESHOT125,PWM400};
	float duty[] = {1500,1501,1502,1500.2,1500.45,1500.4,1500,2000};
	while(1)
	{
		//set_pwm();
		actuator_fpga.setPWM(8,_mode,duty);
		usleep(2500);

	}
}


void start_actuator_fpga(void)
{


  bool ret = create_thread("actuator_fpga", thread_actuator_fpga, NULL);


}

// void PCA9685::setPWM(uint8_t channel, const uint16_t &value)
// {
// 	if (value >= 4096) {
// 		printf("invalid pwm value_2\n\r");
// 		return;
// 	}

// 	uint8_t buf[5] = {};
// 	buf[0] = PCA9685_REG_LED0 + channel * PCA9685_REG_LED_INCREMENT;
// 	buf[1] = 0x00;
// 	buf[2] = 0x00;
// 	buf[3] = (uint8_t)(value & (uint8_t)0xFF);
// 	buf[4] = value != 0 ? ((uint8_t)(value >> (uint8_t)8)) : PCA9685_LED_ON_FULL_ON_OFF_MASK;

// 	int ret = i2c_pca9685.transfer(buf, 5, nullptr, 0);

// 	if (OK != ret) {
// 		printf("setPWM: i2c::transfer returned %d\n", ret);
// 	}else{
// 		printf("pwm set ok %d, %d\n\r",buf[3],buf[4]);
// 	}
// }