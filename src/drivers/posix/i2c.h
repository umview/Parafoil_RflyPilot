#ifndef _I2C_H_
#define _I2C_H_


// #include <string.h>
// #include <sys/time.h>
// #include <malloc.h>
// #include <math.h>
// #include <stdlib.h>
// #include <sys/types.h>
// #include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

class i2c
{
private:
    /* 
    * i2c handle returned by open 
    */
    int	_fd{-1};
    /* 
    * device address 
    */
    unsigned char _addr;
public:
    /* 
    * number of retries 
    */
    uint8_t		_retries{0};
    /* 
    * push <device address> to <_addr> for <tranfer()> 
    */
    i2c(unsigned char device_addr); 
    /* 
    * open i2c device according to path <i2c_dev> 
    */
    int init(const char *i2c_dev);
    /* 
    * transfer data 
    */
    int transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);
    /*
    *
    */
    void set_device_address(unsigned char address){_addr = address;}

};



#endif