#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
// #include <malloc.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "posix/i2c.h"

// namespace drv_pca9685_pwm
// {

#define PCA9685_REG_MODE1 0x00			// Mode  register  1
#define PCA9685_REG_MODE2 0x01			// Mode  register  2
#define PCA9685_REG_SUBADR1 0x02		// I2C-bus subaddress 1
#define PCA9685_REG_SUBADR2 0x03		// I2C-bus subaddress 2
#define PCA9685_REG_SUBADR3 0x04		// I2C-bus subaddress 3
#define PCA9685_REG_ALLCALLADR 0x05     // LED All Call I2C-bus address
#define PCA9685_REG_LED0 0x06			// LED0 start register
#define PCA9685_REG_LED0_ON_L 0x06		// LED0 output and brightness control byte 0
#define PCA9685_REG_LED0_ON_H 0x07		// LED0 output and brightness control byte 1
#define PCA9685_REG_LED0_OFF_L 0x08		// LED0 output and brightness control byte 2
#define PCA9685_REG_LED0_OFF_H 0x09		// LED0 output and brightness control byte 3
#define PCA9685_REG_LED_INCREMENT 4 	// compute the other 15 channels
#define PCA9685_REG_ALLLED_ON_L 0xFA    // load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define PCA9685_REG_ALLLED_ON_H 0xFB	// load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define PCA9685_REG_ALLLED_OFF_L 0xFC	// load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define PCA9685_REG_ALLLED_OFF_H 0xFD	// load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PCA9685_REG_PRE_SCALE 0xFE		// prescaler for output frequency
#define PCA9685_REG_TESTMODE 0xFF       // test mode register

// Mode register 1
#define PCA9685_MODE1_RESTART_MASK 0x80
#define PCA9685_MODE1_EXTCLK_MASK 0x40
#define PCA9685_MODE1_AI_MASK 0x20
#define PCA9685_MODE1_SLEEP_MASK 0x10
#define PCA9685_MODE1_SUB1_MASK 0x08
#define PCA9685_MODE1_SUB2_MASK 0x04
#define PCA9685_MODE1_SUB3_MASK 0x02
#define PCA9685_MODE1_ALLCALL_MASK 0x01

// Mode register 2
#define PCA9685_MODE2_INVRT_MASK 0x10
#define PCA9685_MODE2_OCH_MASK 0x08
#define PCA9685_MODE2_OUTDRV_MASK 0x04
#define PCA9685_MODE2_OUTNE_MASK 0x03

// LED_ON, LED_OFF control registers
#define PCA9685_LED0_ON_L_MASK 0xFF
#define PCA9685_LED_ON_FULL_ON_OFF_MASK 0x10
#define PCA9685_LED0_ON_H_MASK 0x0F

// PRE_SCALE register
#define PCA9685_PRE_SCALE_MASK 0xFF

#define PCA9685_PWM_CHANNEL_COUNT 16
#define PCA9685_PWM_RES 4096        //Resolution 4096=12bit
/* This should be 25000000 ideally,
 * but it seems most chips have its oscillator working at a higher frequency
 * Reference: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/6664ce936210eea53259b814062009d9569a4213/Adafruit_PWMServoDriver.h#L66 */
#define PCA9685_CLOCK_INT 24450000.0 //25MHz internal clock  26075000.0 27777778 25000000

#define PCA9685_CLOCK_EXT 25000000.0

// #ifndef PCA9685_CLOCK_EXT
// #define PCA9685_CLOCK_FREQ PCA9685_CLOCK_INT   // use int clk
// #else
// #define PCA9685_CLOCK_FREQ PCA9685_CLOCK_EXT   // use ext clk
// #endif

#define PCA9685_DEVICE_BASE_PATH	"/dev/i2c-1"
#define PWM_DEFAULT_FREQUENCY 50    // default pwm frequency
#define PCA9685_DEVICE_ADDRESS 0x40

class PCA9685
{
public:

    class i2c i2c_pca9685;


    uint8_t DEFAULT_MODE1_CFG;
    float PCA9685_CLOCK_FREQ;
    int Stop();

    /*
    * outputs formatted to us.
    */
    int updatePWM(const float *outputs, unsigned num_outputs);

    int setFreq(float freq);

    int initReg();
    /*
    * disable all of the output
    */
    void disableAllOutput();

    /*
    * turn off oscillator
    */
    void stopOscillator();

    /*
    * turn on oscillator
    */
    void startOscillator();

    /*
    * turn on output
    */
    void triggerRestart();

    inline float getFrequency() {return _Freq;}

    /* 
    * pca9685 init
    */
    int pca9685_init(const char * dev_path, float pwmFreq, bool extClock);

    /*
    * Init i2c-1 device
    */
    // int init();
protected:
    /**
     * set PWM value for a channel[0,15].
     * value should be range of 0-4095
     */
    void setPWM(uint8_t channel, const uint16_t &value);

    /**
     * set all PWMs in a single I2C transmission.
     * value should be range of 0-4095
     */
    void setPWM(uint8_t channel_count, const uint16_t *value);

    /*
    * set clock divider
    */
    void setDivider(uint8_t value);
    // int probe() override;
    /**
     * Perform an I2C transaction to the device.
     *
     * At least one of send_len and recv_len must be non-zero.
     *
     * @param send		Pointer to bytes to send.
     * @param send_len	Number of bytes to send.
     * @param recv		Pointer to buffer for bytes received.
     * @param recv_len	Number of bytes to receive.
     * @return		OK if the transfer was successful, -errno
     *			otherwise.
    */
    // int transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);
    // #ifdef PCA9685_CLOCK_EXT
    //     static const uint8_t DEFAULT_MODE1_CFG = 0x70;  // Auto-Increment, Sleep, EXTCLK
    // #else
    //     static const uint8_t DEFAULT_MODE1_CFG = 0x30;  // Auto-Increment, Sleep
    // #endif
    static const uint8_t DEFAULT_MODE2_CFG = 0x04;  // totem pole

    float _Freq = PWM_DEFAULT_FREQUENCY;
    // uint8_t		_retries{0};

private:
    // int	_fd{-1};
};
extern class PCA9685 pca9685_dev;
extern class PCA9685 pca9685_dev_aux;
// }
unsigned char _i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo);
unsigned char _i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo);


#endif