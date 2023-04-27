#include "pca9685.h"

#define PX4_ERROR (-1)
#define PX4_OK 0
#define ENOENT    2
#define OK 0
#define EINVAL   22

// using namespace drv_pca9685_pwm;
// using namespace std;
class PCA9685 pca9685_dev;
class i2c i2c_pca9685(PCA9685_DEVICE_ADDRESS);

int PCA9685::pca9685_init(float pwmFreq, bool extClock)
{
	int ret = 0;
	ret = i2c_pca9685.init(PCA9685_DEVICE_BASE_PATH);
    if (OK != ret) {
        printf("i2c_pca9685.init failed (%i)", ret);
		return ret;
	}
	//set clock
	if(extClock){
		PCA9685_CLOCK_FREQ = 25000000.0;
		DEFAULT_MODE1_CFG = 0x70; // Auto-Increment, Sleep, EXTCLK
	}
	else{
		DEFAULT_MODE1_CFG = 0x30; // Auto-Increment, Sleep
		PCA9685_CLOCK_FREQ = 24450000.0; //25MHz internal clock  26075000.0 27777778 25000000
	}
	
    disableAllOutput();
    usleep(5000);
    triggerRestart();
    usleep(5000);
    stopOscillator();
    usleep(50000);
    ret = initReg();
    if (OK != ret) {
		return ret;
	}
    ret = setFreq(pwmFreq);//setFreq = Freq/0.915?
    if (OK != ret) {
		return ret;
	}
    printf("PWM freq: %f\n\r", getFrequency());
    startOscillator();
    usleep(5000);
	return ret;
}

int PCA9685::Stop()
{
	disableAllOutput();
	stopOscillator();
	return PX4_OK;
}

int PCA9685::updatePWM(const float *outputs, unsigned num_outputs)
{
	if (num_outputs > PCA9685_PWM_CHANNEL_COUNT) {
		num_outputs = PCA9685_PWM_CHANNEL_COUNT;
		printf("PCA9685 can only drive up to 16 channels\n");
	}

	float out[PCA9685_PWM_CHANNEL_COUNT];
	memcpy(out, outputs, sizeof(float) * num_outputs);
	uint16_t out2[PCA9685_PWM_CHANNEL_COUNT];
	for (unsigned i = 0; i < num_outputs; ++i) {
		out2[i] = (uint16_t)roundl((out[i] * _Freq * PCA9685_PWM_RES / (float)1e6)); // convert us to 12 bit resolution
	}
	// printf("%d,, %d,, %d,, %d,,\n\r",out[0],out[1],out[2],out[3]);

	setPWM(num_outputs, out2);

	return 0;
}

int PCA9685::setFreq(float freq)
{
	uint16_t realResolution = floorl((float)PCA9685_CLOCK_FREQ / freq);

	if (realResolution < PCA9685_PWM_RES) { // unable to provide enough resolution
		printf("frequency too high\n");
		return -EINVAL;
	}

	uint16_t divider = (uint16_t)round((float)PCA9685_CLOCK_FREQ / freq / PCA9685_PWM_RES) - 1;
	printf("PCA9685 Divider is %d\n", divider);

	if (divider > 0x00FF) { // out of divider
		printf("frequency too low\n");
		return -EINVAL;
	}

	float freq_err = ((float)PCA9685_CLOCK_FREQ / (float)(divider + (uint16_t)1)
			  - (float)(freq * PCA9685_PWM_RES))
			 / (float)(freq * PCA9685_PWM_RES); // actually asked for (freq * PCA9685_PWM_RES)

	if (fabsf(freq_err) > 0.01f) { // TODO decide threshold
		printf("Frequency error too large: %.4f\n", (double)freq_err);
		// should we return an error?
	}

	_Freq = (float)PCA9685_CLOCK_FREQ / (float)(divider + (uint16_t)1) / PCA9685_PWM_RES; // use actual pwm freq instead.

	setDivider(divider);

	return PX4_OK;

}

int PCA9685::initReg()
{
	uint8_t buf[2] = {};

	buf[0] = PCA9685_REG_MODE1;
	buf[1] = DEFAULT_MODE1_CFG;
	int ret = i2c_pca9685.transfer(buf, 2, nullptr, 0); // make sure oscillator is disabled

	if (OK != ret) {
		printf("1 init: PCA9685::transfer returned %d\n", ret);
		return ret;
	}

	ret = i2c_pca9685.transfer(buf, 2, nullptr, 0); // enable EXTCLK if possible

	if (OK != ret) {
		printf("2 init: PCA9685::transfer returned %d\n", ret);
		return ret;
	}

	buf[0] = PCA9685_REG_MODE2;
	buf[1] = DEFAULT_MODE2_CFG;
	ret = i2c_pca9685.transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		printf("init: i2c::transfer returned %d\n", ret);
		return ret;
	}

	return PX4_OK;
}

void PCA9685::setPWM(uint8_t channel, const uint16_t &value)
{
	if (value >= 4096) {
		printf("invalid pwm value_2\n\r");
		return;
	}

	uint8_t buf[5] = {};
	buf[0] = PCA9685_REG_LED0 + channel * PCA9685_REG_LED_INCREMENT;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = (uint8_t)(value & (uint8_t)0xFF);
	buf[4] = value != 0 ? ((uint8_t)(value >> (uint8_t)8)) : PCA9685_LED_ON_FULL_ON_OFF_MASK;

	int ret = i2c_pca9685.transfer(buf, 5, nullptr, 0);

	if (OK != ret) {
		printf("setPWM: i2c::transfer returned %d\n", ret);
	}else{
		printf("pwm set ok %d, %d\n\r",buf[3],buf[4]);
	}
}

void PCA9685::setPWM(uint8_t channel_count, const uint16_t *value)
{
	uint8_t buf[PCA9685_PWM_CHANNEL_COUNT * PCA9685_REG_LED_INCREMENT + 1] = {};
	buf[0] = PCA9685_REG_LED0;

	for (int i = 0; i < channel_count; ++i) {
		if (value[i] >= 4096) {
			printf("invalid pwm value_1\n\r");
			return;
		}

		buf[1 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
		buf[2 + i * PCA9685_REG_LED_INCREMENT] = 0x00;
		buf[3 + i * PCA9685_REG_LED_INCREMENT] = (uint8_t)(value[i] & (uint8_t)0xFF);
		buf[4 + i * PCA9685_REG_LED_INCREMENT] = value[i] != 0 ? ((uint8_t)(value[i] >> (uint8_t)8)) :
				PCA9685_LED_ON_FULL_ON_OFF_MASK;
		// printf("%d, %d\n\r",buf[3 + i * PCA9685_REG_LED_INCREMENT],buf[4 + i * PCA9685_REG_LED_INCREMENT]);
	}

	int ret = i2c_pca9685.transfer(buf, channel_count * PCA9685_REG_LED_INCREMENT + 1, nullptr, 0);

	if (OK != ret) {
		printf("setPWM: i2c::transfer returned %d\n\r", ret);
	}
}

void PCA9685::disableAllOutput()
{
	uint8_t buf[5] = {};
	buf[0] = PCA9685_REG_ALLLED_ON_L;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = PCA9685_LED_ON_FULL_ON_OFF_MASK;

	int ret = i2c_pca9685.transfer(buf, 5, nullptr, 0);

	if (OK != ret) {
		printf("i2c::transfer returned %d\n", ret);
	}
}

void PCA9685::setDivider(uint8_t value)
{
	uint8_t buf[2] = {};
	buf[0] = PCA9685_REG_PRE_SCALE;
	buf[1] = value;
	int ret = i2c_pca9685.transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		printf("i2c::transfer returned %d\n\r", ret);
		return;
	}
}

void PCA9685::startOscillator()
{
	uint8_t buf[2] = {PCA9685_REG_MODE1};

	// clear sleep bit, with restart bit = 0
	buf[1] = DEFAULT_MODE1_CFG & (~PCA9685_MODE1_SLEEP_MASK);
	int ret = i2c_pca9685.transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		printf("startOscillator: i2c::transfer returned %d\n\r", ret);
		return;
	}
}

void PCA9685::stopOscillator()
{
	uint8_t buf[2] = {PCA9685_REG_MODE1};

	// set to sleep
	buf[1] = DEFAULT_MODE1_CFG;
	int ret = i2c_pca9685.transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		printf("i2c::transfer returned %d\n\r", ret);
		return;
	}
}

void PCA9685::triggerRestart()
{
	uint8_t buf[2] = {PCA9685_REG_MODE1};

	// clear sleep bit, with restart bit = 0
	buf[1] = DEFAULT_MODE1_CFG & (~PCA9685_MODE1_SLEEP_MASK);
	buf[1] |= PCA9685_MODE1_RESTART_MASK;
	int ret = i2c_pca9685.transfer(buf, 2, nullptr, 0);

	if (OK != ret) {
		printf("pca9685 triggerRestart: i2c::transfer returned %d\n\r", ret);
		return;
	}
}



/*****************************************************************************
    延时函数（ms）
******************************************************************************/
void _alpu_delay_ms(unsigned int i)  
{
    usleep(2000 * i);
}
/*****************************************************************************
    i2c读函数，参数1：从设备地址，参数2：寄存器地址，参数3：读取数据缓冲区，参数4：读取数据大小
******************************************************************************/
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
    int fd, ret;
    unsigned char buftmp[32];
    struct i2c_rdwr_ioctl_data i2c_data;
    const char      *i2c_dev = "/dev/i2c-1";
    //----------------------------------

    // device_addr >>= 1;
    //init
    fd = open(i2c_dev, O_RDWR);
    if (fd<0)
    {
        printf("not have /dev/i2c-1 t\r\n");
        return -1;
    }

    i2c_data.nmsgs = 2;
    i2c_data.msgs = (struct i2c_msg *)malloc(i2c_data.nmsgs *sizeof(struct i2c_msg));
    if (i2c_data.msgs == NULL)
    {
        printf("malloc error");
        close(fd);
        return -1;
    }

    ioctl(fd, I2C_TIMEOUT, 1);
    ioctl(fd, I2C_RETRIES, 2);

    //write reg
    memset(buftmp, 0, 32);
    buftmp[0] = sub_addr;
    i2c_data.msgs[0].len = 1;
    i2c_data.msgs[0].addr = device_addr;
    i2c_data.msgs[0].flags = 0;     // 0: write 1:read
    i2c_data.msgs[0].buf = buftmp;
    //read data
    i2c_data.msgs[1].len = ByteNo;
    i2c_data.msgs[1].addr = device_addr;
    i2c_data.msgs[1].flags = 1;     // 0: write 1:read
    i2c_data.msgs[1].buf = buff;


    ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
    if (ret < 0)
    {
        printf("read data %x %x error\r\n", device_addr, sub_addr);
        close(fd);
        free(i2c_data.msgs);
        return 1;
    }
    free(i2c_data.msgs);
    close(fd);

#if 1
    int i;
    printf("i2c__read 0x%02x:",buftmp[0]);
    for (i = 0; i < ByteNo; i++)
    {
    printf(" 0x%02x",buff[i]);
    }
    printf("\n");
#endif

    return 0;
}

/*****************************************************************************
    i2c写函数，参数1：从设备地址，参数2：寄存器地址，参数3：要写入的数据缓冲区，参数4：写入数据大小
******************************************************************************/
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
    int fd, ret;
    unsigned char buftmp[32];
    struct i2c_rdwr_ioctl_data i2c_data;
    const char      *i2c_dev = "/dev/i2c-1";
    //----------------------------------

    // device_addr >>= 1;
    //init
    fd = open(i2c_dev, O_RDWR);
    if (fd < 0)
    {
        printf("not have /dev/i2c-1\r\n");
        return -1;
    }

    i2c_data.nmsgs = 1;
    i2c_data.msgs = (struct i2c_msg *)malloc(i2c_data.nmsgs *sizeof(struct i2c_msg));
    if (i2c_data.msgs == NULL)
    {
        printf("malloc error");
        close(fd);
        return -1;
    }

    ioctl(fd, I2C_TIMEOUT, 1);
    ioctl(fd, I2C_RETRIES, 2);

    memset(buftmp, 0, 32);
    buftmp[0] = sub_addr;
    memcpy(buftmp + 1, buff, ByteNo);
    i2c_data.msgs[0].len = ByteNo + 1;;
    i2c_data.msgs[0].addr = device_addr;
    i2c_data.msgs[0].flags = 0;     // 0: write 1:read
    i2c_data.msgs[0].buf = buftmp;
    ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
    if (ret < 0)
    {
        printf("write reg %x %x error\r\n", device_addr, sub_addr);
        close(fd);
        free(i2c_data.msgs);
        return 1;
    }
    free(i2c_data.msgs);
    close(fd);

#if 1
    int i;
    printf("i2c_write 0x%02x:",buftmp[0]);
    for(i=0; i<ByteNo; i++)
    {
    printf(" 0x%02x",buftmp[1+i]);
    }
    printf("\n");
#endif
    _alpu_delay_ms(100);
    return 0;
}




