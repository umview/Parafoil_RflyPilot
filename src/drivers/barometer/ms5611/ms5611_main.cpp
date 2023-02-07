/*
* @Compile and Link: gcc -DTESTMODE -o main ms5611_main.cpp ms5611.cpp posix/i2c.cpp px4lib/test_time.cpp -lpthread -lstdc++
* @run: <sudo taskset -c 2 ./main> this will make our program run on core 2
*/


#include "ms5611.h"


int main()
{
    timespec main_sleep;
    main_sleep.tv_sec = 1;
    main_sleep.tv_nsec = 0;
    // c_ms5611.init();
    start_baro();
    while (1)
    {
        // c_ms5611.RunImpl();
        printf("%f %f %d\n\r",c_ms5611._pressure, c_ms5611._temperature, c_ms5611._timestamp_sample);
        // usleep(500*1000);
        nanosleep(&main_sleep,NULL);
    }
    
    return 0;
}
