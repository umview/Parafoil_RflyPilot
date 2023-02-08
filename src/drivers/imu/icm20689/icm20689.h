#ifndef _ICM20689_H_
#define _ICM20689_H_
#include "include.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <stdint.h>
// #include <unistd.h>

// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <linux/spi/spidev.h>

  typedef struct{
    float gyro_xyz[3];
    float accel_xyz[3];
  }icm20689_data_typedef;
//Init spi0.0
int icm20689_spi_init(void);
//Read Sensor Data
int icm20689_read(float *sensorData);

void *thread_icm20689(void *ptr);
void start_icm20689(void);
bool icm20689_recv_spi(icm20689_data_typedef * payload);
extern class adaptive_delay_typedef icm20689_delay;

#endif