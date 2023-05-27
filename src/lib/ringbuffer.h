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
extern char time_buf[50];

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
    char path[200];
    int fp;
    int ret;
    uint32_t div;
    uint32_t div_count;
    // ringbuffer_typedef(int _size)
    // {
    //     size = _size;
    //     publish_rate_hz = 1;
    //     buffer = new data_type[size];
    //     index = 0;
    //     if(pthread_spin_init(&spinlock,PTHREAD_PROCESS_PRIVATE))
    //     {
    //         printf("error spin lock init failed\n");
    //     }
    //     log_enable = false;
    //     msg_name = NULL;
    // }    
    // ringbuffer_typedef(int _size, char const *_msg_name)
    // {
    //     size = _size;
    //     publish_rate_hz = 1;
    //     buffer = new data_type[size];
    //     index = 0;
    //     msg_name = _msg_name;
    //     if(pthread_spin_init(&spinlock,PTHREAD_PROCESS_PRIVATE))
    //     {
    //         printf("error spin lock init failed\n");
    //     }
    //     log_enable = false;
    // }
    ringbuffer_typedef(int _size, char const *_msg_name, uint32_t _div)
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
        div = _div;
        div_count = 0;
    }    

    void log_write(data_type *data)
    {
        static bool log_file_init = false;
        if(log_file_init == false)
        {
            log_file_init = true;
            create_log_file(".");//将日志文件写入到当前文件夹

        }
        ret = write(fp,(uint8_t*)data,sizeof(data_type));        
    }


    void create_log_file(const char * dir)
    {
        char syspath[100] = {0};
        sprintf(syspath, "%s/log-%s", (char*)dir, time_buf);    
        struct stat st = {0};  
        if (stat(syspath, &st) == -1) {
            mkdir(syspath, 0777);
        }
        sprintf(path, "%s/%s.bin",syspath,msg_name);
        fp=open(path, O_RDWR | O_CREAT,0777);
        if(fp == -1){
                printf("open failed!\n");
        }else{
                printf("open success!\n");
        }
        printf("msg log created : size of msg : %s is %d\n",msg_name, sizeof(data_type));
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
          //if(deltaT < 0.00001)deltaT = 0.00001;
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
        if(div > 0)
        {
            div_count ++;
            if(div_count == div)//日志记录分频
            {
                log_write(data);
                div_count = 0;
            }
        }
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
            return false;
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