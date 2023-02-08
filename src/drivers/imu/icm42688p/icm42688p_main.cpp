/*
* Used for validating qmc5883l driver
* @compile&link: 
gcc -DTESTMODE  -o main icm42688p_main.cpp ICM42688P.cpp ../px4lib/test_time.cpp ../accelerometer/PX4Accelerometer.cpp ../gyroscope/PX4Gyroscope.cpp ../posix/spi.cpp -lpthread -lstdc++ -lm
-DTESTMODE represents add "#define TESTMODE"
* @run: <sudo taskset -c 2 ./main> this will make our program run on core 2
* @using: Run start_qmc5883l() to get mag data in class QMC5883l which is declared in <qmc5883l.h>;
*/

#include "ICM42688P.h"

int main()
{
    timespec main_sleep;
    main_sleep.tv_sec = 1;
    main_sleep.tv_nsec = 0;
    start_icm42688p();
    while (1)
    {
        // printf("%f, %f, %f, %d \n\r", qmc5883l.magData[0], qmc5883l.magData[1], qmc5883l.magData[2], qmc5883l.timestamp[0]);
        nanosleep(&main_sleep,NULL);
    }
    
    return 0;
}