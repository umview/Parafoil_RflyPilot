/*
* Used for validating ist8310 driver
* @compile&link: gcc -DTESTMODE -o main ist8310_main.cpp ist8310.cpp ../posix/i2c.cpp -lpthread -lstdc++
* @run: <sudo taskset -c 2 ./main> this will make our program run on core 2
* @using: Run start_mag() to get mag data in class ist8310 which is declared in <ist8310.h>;
*/

#include <pthread.h>
#include "ist8310.h"


int main()
{
    timespec main_sleep;
    main_sleep.tv_sec = 1;
    main_sleep.tv_nsec = 0;
    start_mag();
    while (1)
    {
        printf("%f, %f, %f, %d \n\r", ist8310.magData[0], ist8310.magData[1], ist8310.magData[2], ist8310.timestamp[0]);
        nanosleep(&main_sleep,NULL);
    }
    
    return 0;
}