#ifndef _RINGBUFFER_
#define _RINGBUFFER_
#include "include.h"
#include "systime.h"
#include <sys/stat.h>
#include <sys/types.h>
// typedef struct 
// {
// 	uint64_t timestamp;
// 	float accel[3];
// }sensor_struct_typedef;
extern uint64_t get_time_now(void);

template <typename data_type>
class ringbuffer_typedef
{
public:
    float publish_rate_hz;
    float deltaT;
    uint64_t t0_us;
    uint64_t t1_us;
    char const *msg_name;
    char path[50];
    bool log_enable;
    int fp;
    int ret;
    ringbuffer_typedef(int _size)
    {
        size = _size;
        publish_rate_hz = 1;
        buffer = new data_type[size];
        index = 0;
        if(pthread_spin_init(&spinlock,PTHREAD_PROCESS_PRIVATE))
        {
            printf("error spin lock init failed\n");
        }
        log_enable = false;
        msg_name = NULL;
    }    
    ringbuffer_typedef(int _size, char const *_msg_name)
    {
        size = _size;
        publish_rate_hz = 1;
        buffer = new data_type[size];
        index = 0;
        msg_name = _msg_name;
        if(pthread_spin_init(&spinlock,PTHREAD_PROCESS_PRIVATE))
        {
            printf("error spin lock init failed\n");
        }
        log_enable = false;
    }
    ringbuffer_typedef(int _size, char const *_msg_name, bool _log_enable)
    {
        size = _size;
        publish_rate_hz = 1;
        buffer = new data_type[size];
        index = 0;
        msg_name = _msg_name;
        if(pthread_spin_init(&spinlock,PTHREAD_PROCESS_PRIVATE))
        {
            printf("error spin lock init failed\n");
        }
        log_enable = _log_enable;
        // static struct tm mytm={0};
        // static struct tm* p_tm;
        // static time_t time;
        // p_tm = localtime_r(&time,&mytm);        
        // sprintf(path, "/dev/shm/log/");//, p_tm->tm_mon + 1, p_tm->tm_mday, p_tm->tm_hour, p_tm->tm_min);            
        // if(mkdir("/dev/shm/log/", 0777) == -1)
        // {
        //     printf("mkdir %s error\n",msg_name);
        // }
        // sprintf(path, "/dev/shm/log/%s.bin",msg_name);//, p_tm->tm_mon + 1, p_tm->tm_mday, p_tm->tm_hour, p_tm->tm_min, msg_name);
        // fp=open(path, O_RDWR | O_CREAT,0777);
        // if(fp == -1){
        //         printf("open failed!\n");
        // }else{
        //         printf("open success!\n");
        // }
        // printf("size of msg : %s is %d\n",msg_name, sizeof(data_type));
    }    

    void log_write(data_type *data)
    {

        //将数组a的内容写入到文件
        ret = write(fp,(uint8_t*)data,sizeof(data_type));        
    }




    void lock(void)
    {
        if(err = pthread_spin_lock(&spinlock))
        {
            printf("lock err :  %d\n",err);
        }
    }
    void unlock(void)
    {
        if(err = pthread_spin_unlock(&spinlock))
        {
            printf("unlock err :  %d\n",err);
        }        
    }
    // void show(void)
    // {
    //     for(int i = 0; i < size; i++)
    //     {
    //         printf("index %d, time: %d, accel %f %f %f\n",i,buffer[i].timestamp, buffer[i].accel[0], buffer[i].accel[1], buffer[i].accel[2]);
    //     }
    //     //printf("\n");
    // }
    void calc_publish_rate(void)
    {
          static bool initd = false;        
          if(!initd)
          {
            initd = true;
            t0_us = get_time_now();
          }
          t1_us = get_time_now();
          deltaT = (float)(t1_us - t0_us) / 1e6;
          // if(deltaT > 1) deltaT = 1;
          // else if(deltaT < 0.00001)deltaT = 0.00001;
          publish_rate_hz = 1 / deltaT;
          t0_us = t1_us;
    }
    void publish(data_type *data)
    {
        int _index = 0;
        calc_publish_rate();
        lock();
        _index = index;
        unlock();
        if((_index + 1) == size)
        {
            buffer[0] = *data;
            lock();
            index = 0;
            unlock();
        }else{
            buffer[_index + 1] = *data;
            lock();
            index += 1;
            unlock();
        }
        //if(log_enable)log_write(data);
    }
    // bool readN(data_type *data,int length)
    // {
    //     static int _index = 0;
    //     if(length > size)
    //     {
    //         printf("lenght > buffer size !!!!\n");
    //         return false;
    //     }
    //     lock();
    //     _index = index;
    //     unlock();
    //     for(int i = (length - 1); i >= 0; i--)
    //     {
    //         data[i] = buffer[_index];
    //         if((_index - 1) == -1)
    //         {
    //             _index = size - 1;
    //         }else{
    //             _index --;
    //         }
    //     }
    //     return true;

    // }
    bool read(data_type *data)
    {
        static int _index = 0;
        static data_type tmp;
        lock();
        _index = index;
        unlock();
        tmp = buffer[_index];
        if(tmp.timestamp > data[0].timestamp)
        {
            data[0] = tmp;
            return true;
        }else{
            data[0] = tmp;
            return true;
        }
        // data[0] = tmp;
        // return true;
    }    
private:
    pthread_spinlock_t spinlock;
    data_type *buffer;
    int index;
    int size;
    int err;
};




#endif