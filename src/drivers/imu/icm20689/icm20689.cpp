#include "icm20689.h"

#define spi0_path        "/dev/spidev0.0"//前面的0表示SPI1，后面的0表示片选引脚0

#define ACCEL_XOUT_H 0x3B
#define FIFO_EN     0x23
#define PWR_MGMT_1  0x68
#define USER_CTRL   0x6A
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W    0x74
#define WHO_AM_I    0x75

/*SPI 接收 、发送 缓冲区*/
unsigned char tx_buffer[20];
unsigned char rx_buffer[20];

static int fd;
static uint32_t mode = SPI_MODE_3;    // 用于保存 SPI 工作模式
// static uint8_t mode = SPI_MODE_3;
static uint8_t bits = 8;                // 接收、发送数据位数
static uint32_t speed = 10000000;        // 发送速度
static uint16_t _delay;                  //保存延时时间

//spi发送数据
static int transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len);
static int spi_reg_read(int fd, uint8_t const addr, uint8_t *data);
static int spi_reg_write(int fd, uint8_t const addr, uint8_t const data);
static int16_t combine(uint8_t msb, uint8_t lsb);


int icm20689_read(float *sensorData)
{
    uint8_t sensor_tx_buf[15] = {ACCEL_XOUT_H|0x80,0x00,};
    uint8_t sensor_rx_buf[15] = {0x00,};
    transfer(fd, sensor_tx_buf, sensor_rx_buf, 15);
    //Accel Data and Gyro Data
    sensorData[0] = combine(sensor_rx_buf[1],sensor_rx_buf[2])/208.76656473;
    sensorData[1] = -combine(sensor_rx_buf[3],sensor_rx_buf[4])/208.76656473;
    sensorData[2] = -combine(sensor_rx_buf[5],sensor_rx_buf[6])/208.76656473;
    sensorData[3] = combine(sensor_rx_buf[9],sensor_rx_buf[10])/938.734;
    sensorData[4] = -combine(sensor_rx_buf[11],sensor_rx_buf[12])/938.734;
    sensorData[5] = -combine(sensor_rx_buf[13],sensor_rx_buf[14])/938.734;

    return 0;
}

int16_t combine(uint8_t msb, uint8_t lsb)
{
    return (msb << 8u) | lsb;
}

int spi_reg_write(int fd, uint8_t const addr, uint8_t const data)
{
    int ret;
    tx_buffer[0] = addr;
    tx_buffer[1] = data;
    ret = transfer(fd, tx_buffer, rx_buffer, 2);
    if (-1 == ret)
    {
        printf("transfer error...\n");
        return -1;
    }
    return 0;
}

int spi_reg_read(int fd, uint8_t const addr, uint8_t *data)
{
    int ret;
    tx_buffer[0] = 0x80|addr;
    tx_buffer[1] = 0x00;
    ret = transfer(fd, tx_buffer, rx_buffer, 2);
    if (-1 == ret)
    {
        printf("transfer error...\n");
        return -1;
    }
    data[0] = rx_buffer[1];
    return 0;
}

int transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .speed_hz = speed,
        .delay_usecs = _delay,
        .bits_per_word = bits,
    };

    ret = ioctl(fd,SPI_IOC_MESSAGE(1),&tr);
    if( ret == -1 )
    {
        return -1;  
    }

    return 0;
}

int icm20689_spi_init(void)
{
    int ret;
    fd = open(spi0_path,O_RDWR);
    if(fd < 0)
    {
        perror("/dev/spidev0.0");
        return -1;
    }

    // 设置spi工作模式
    // SPI_IOC_RD_MODE          读取SPI模式
    // SPI_IOC_WR_MODE          写入SPI模式
    // SPI_IOC_RD_LSB_FIRST     SPI读取数据模式(LSB先行返回1)
    // SPI_IOC_WR_LSB_FIRST     SPI写入数据模式。(0:MSB，非0：LSB)
    // SPI_IOC_RD_BITS_PER_WORD SPI读取设备的字长
    // SPI_IOC_WR_BITS_PER_WORD SPI写入设备的字长
    // SPI_IOC_RD_MAX_SPEED_HZ  读取SPI设备的最大通信频率
    // SPI_IOC_WR_MAX_SPEED_HZ  写入SPI设备的最大通信速率
    // SPI_IOC_MESSAGE(N)       一次进行双向/多次读写操作


    ret = ioctl(fd,SPI_IOC_WR_MODE,&mode);
    if( ret == -1)
    {
        printf("SPI_IOC_WR_MODE error......\n ");
        goto fd_close;  
    }
    printf("write_SPI_MODE: %d\n\r ", mode);
    ret = ioctl(fd,SPI_IOC_RD_MODE,&mode);
    if( ret == -1)
    {
        printf("SPI_IOC_RD_MODE error......\n ");
        goto fd_close;
    }
    printf("read_SPI_MODE: %d\n\r ", mode);

   //设置SPI通信的字长
    ret = ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&bits);
    if( ret == -1)
    {
        printf("SPI_IOC_WR_BITS_PER_WORD error......\n ");
        goto fd_close;
    }
    ret = ioctl(fd,SPI_IOC_RD_BITS_PER_WORD,&bits);
    if( ret == -1)
    {
        printf("SPI_IOC_RD_BITS_PER_WORD error......\n ");
        goto fd_close;
    }


   //设置SPI最高工作频率
    ret = ioctl(fd,SPI_IOC_WR_MAX_SPEED_HZ,&speed);
    if( ret == -1)
    {
        printf("SPI_IOC_WR_MAX_SPEED_HZ error......\n ");
        goto fd_close;
    }
    ret = ioctl(fd,SPI_IOC_RD_MAX_SPEED_HZ,&speed);
    if( ret == -1)
    {
        printf("SPI_IOC_RD_MAX_SPEED_HZ error......\n ");
        goto fd_close;
    }


    printf("spi mode: %d\n\r", mode);
    printf("bits per word: %d\n\r", bits);
    printf("max speed: %d Hz (%d KHz)\n\r", speed, speed / 1000);

    return 0;

fd_close:

    close(fd);
    return -1;
}

class adaptive_delay_typedef icm20689_delay(0.5,15,50);
void *thread_icm20689(void *ptr)
{
    timespec thread_icm20689_sleep;
    thread_icm20689_sleep.tv_sec = 0;
    thread_icm20689_sleep.tv_nsec = 2*1000*1000;
    
    icm20689_data_typedef icm20689_data; 
    gyro_raw_typedef _gyro_raw;
    gyro_typedef _gyro;
    accel_raw_typedef _accel_raw;
    accel_typedef _accel;
   
    uint64_t time_us;  
    int i = 0;  

    class iir_lpf2_typedef accel_lpf[3];
    class iir_lpf2_typedef gyro_lpf[3];
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);
    for(int i = 0; i < 3; i ++)
    {
        accel_lpf[i].set_cutoff_frequency(config.imu_rate, config.accel_cutoff_hz);
        gyro_lpf[i].set_cutoff_frequency(config.imu_rate, config.gyro_cutoff_hz);        
    }

    core_bind(3);
    icm20689_spi_init();
    while (1)
    {

          if(icm20689_recv_spi(&icm20689_data))
          {
            time_us = get_time_now();
            // _accel_raw.timestamp = time_us;

            // for(i = 0; i < 3; i++)
            // {
            //     _accel_raw.accel[i] = icm20689_data.accel_xyz[i];
            //     _gyro_raw.gyro[i] = icm20689_data.gyro_xyz[i];
            // }
            // // calibration.apply_accel_calibration(_imu_raw.accel, _imu.accel);
            // calibration.apply_accel_calibration(_accel_raw.accel, _accel.accel);
            // _accel.timestamp = time_us;
            // accel_raw_msg.publish(&_accel_raw);
            // accel_msg.publish(&_accel);
            // _gyro_raw.timestamp = time_us;
            // // calibration.apply_accel_calibration(_imu_raw.accel, _imu.accel);
            // calibration.apply_gyro_calibration(_gyro_raw.gyro, _gyro.gyro);
            // _gyro.timestamp = time_us;
            // gyro_raw_msg.publish(&_gyro_raw);
            // gyro_msg.publish(&_gyro);
             //printf("accel %f %f %f\n",icm20689_data.accel_xyz[0],icm20689_data.accel_xyz[1],icm20689_data.accel_xyz[2]);
             //printf("gyro %f %f %f\n",icm20689_data.gyro_xyz[0],icm20689_data.gyro_xyz[1],icm20689_data.gyro_xyz[2]);

              _gyro_raw.timestamp = _accel_raw.timestamp = time_us;
              _gyro.timestamp = _accel.timestamp = time_us;
              for(int i = 0; i < 3; i++)
              {
                _accel_raw.accel[i] = accel_lpf[i].apply(icm20689_data.accel_xyz[i]);
                _gyro_raw.gyro[i] = gyro_lpf[i].apply(icm20689_data.gyro_xyz[i]);
              }
              calibration.apply_accel_calibration(_accel_raw.accel, _accel.accel);
              calibration.apply_gyro_calibration(_gyro_raw.gyro, _gyro.gyro);
            accel_raw_msg.publish(&_accel_raw);
            accel_msg.publish(&_accel);
            gyro_raw_msg.publish(&_gyro_raw);
            gyro_msg.publish(&_gyro);
            //printf("accel %f %f %f\n",_accel.accel[0],_accel.accel[1],_accel.accel[2]);

            //printf("OK\n");
            //printf("%x \n", msg_id);
            //printf("recv size %d\n",recvsize);
          }else{
            printf("icm20689 recv failed\n");
          }
        //}
        //nanosleep(&thread_icm20689_sleep,NULL);
        delay_us_combined((uint64_t)(1000000.f / config.imu_rate),&scheduler.imu_flag,&icm20689_delay);

    }
    return NULL;
}
void start_icm20689(void)
{

  bool ret = create_thread("icm20689", thread_icm20689, NULL);

}

bool icm20689_recv_spi(icm20689_data_typedef * payload)
{
  float sensorData[6];
  icm20689_read(sensorData);
  payload->accel_xyz[0] = sensorData[0];
  payload->accel_xyz[1] = sensorData[1];
  payload->accel_xyz[2] = sensorData[2];
  payload->gyro_xyz[0] = sensorData[3];
  payload->gyro_xyz[1] = sensorData[4];
  payload->gyro_xyz[2] = sensorData[5];

  // tmp.accel_xyz[0] = -sensorData[1];
  // tmp.accel_xyz[1] = -sensorData[0];
  // tmp.accel_xyz[2] = sensorData[2];
  // tmp.gyro_xyz[0]  = sensorData[4];
  // tmp.gyro_xyz[1]  = sensorData[3];
  // tmp.gyro_xyz[2]  = -sensorData[5];

  //       sensor_data.s.accel_xyz[0] = -tmp.accel_xyz[1];// * 0.00479;//16*9.81/32768 = 0.0048
  //       sensor_data.s.accel_xyz[1] = -tmp.accel_xyz[0];// * 0.00479;
  //       sensor_data.s.accel_xyz[2] = tmp.accel_xyz[2];// * 0.00479;
  //       sensor_data.s.gyro_xyz[0] = tmp.gyro_xyz[1];// * 0.017453292519;//pi/180
  //       sensor_data.s.gyro_xyz[1] = tmp.gyro_xyz[0];// * 0.017453292519;
  //       sensor_data.s.gyro_xyz[2] = -tmp.gyro_xyz[2];//

  return true;
}